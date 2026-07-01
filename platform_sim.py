import numpy as np
from math_helpers import quaternion_multiply, normalize_quaternion

class MovingPlatform:
    def __init__(self, dt=0.01):
        self.dt = dt
        # State: position (NED), velocity, quaternion (body to NED), angular rates
        self.pos = np.zeros(3)          # North, East, Down (m)
        self.vel = np.array([5.0, 0.0, 0.0])  # m/s (slow ship)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])  # identity
        self.omega = np.zeros(3)        # rad/s body rates
        self.time = 0.0

    def update(self, accel_body, omega_body):
        """Propagate platform state with simple integration (high-fidelity would use better integrators)."""
        # Attitude update (quaternion kinematics)
        omega_quat = np.array([0, *omega_body])
        q_dot = 0.5 * quaternion_multiply(self.quat, omega_quat)
        self.quat = normalize_quaternion(self.quat + q_dot * self.dt)
        self.omega = omega_body

        # Position/velocity (simple, ignore Coriolis for simplicity; add later for fidelity)
        # Rotate accel to NED
        from math_helpers import quaternion_to_rotation_matrix
        R_bn = quaternion_to_rotation_matrix(self.quat)
        accel_ned = R_bn @ accel_body + np.array([0, 0, 9.81])  # gravity
        self.vel += accel_ned * self.dt
        self.pos += self.vel * self.dt
        self.time += self.dt
        return self.pos.copy(), self.vel.copy(), self.quat.copy()
