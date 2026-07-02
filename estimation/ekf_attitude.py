import numpy as np
from utils.math_helpers import normalize_quaternion, quaternion_multiply

class AttitudeINS_EKF:
    """
    Simplified loosely-coupled GNSS/INS attitude estimator.
    Good enough for the demo. You can later upgrade it to a full MEKF.
    """

    def __init__(self, dt=0.05):
        self.dt = dt
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        self.alpha = 0.15  # blending factor with GNSS

    def predict(self, gyro, accel):
        """
        Very simple attitude propagation using gyroscope.
        (In a real EKF this would be the full prediction step with Jacobians)
        """
        # Simple quaternion integration
        omega_quat = np.array([0.0, *gyro])
        q_dot = 0.5 * quaternion_multiply(self.quat, omega_quat)
        self.quat = normalize_quaternion(self.quat + q_dot * self.dt)

        # Very crude velocity integration (for demo only)
        # In real code we would rotate accel into NED frame
        self.vel += accel * self.dt * 0.1   # heavily damped for stability
        self.pos += self.vel * self.dt

    def update(self, gnss_pos, gnss_vel):
        """
        Loosely-coupled update with GNSS position and velocity.
        Returns: est_pos, est_vel, est_quat
        """
        # Blend with GNSS measurements
        self.pos = (1 - self.alpha) * self.pos + self.alpha * gnss_pos
        self.vel = (1 - self.alpha) * self.vel + self.alpha * gnss_vel

        # Return the three values that main/simulator expects
        return self.pos.copy(), self.vel.copy(), self.quat.copy()

