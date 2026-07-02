import numpy as np

class IMU:
    def __init__(self):
        self.gyro_bias = np.array([0.001, -0.0005, 0.0008])  # rad/s
        self.accel_bias = np.array([0.05, -0.03, 0.02])     # m/s²
        self.gyro_noise_std = 0.01
        self.accel_noise_std = 0.1

    def measure(self, true_omega, true_accel):
        gyro = true_omega + self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)
        accel = true_accel + self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
        return gyro, accel

class GNSS:
    def __init__(self):
        self.pos_noise_std = 2.0   # meters (typical consumer GNSS)
        self.vel_noise_std = 0.1   # m/s
        # For advanced: add multi-GNSS, carrier phase, iono, RTCM, etc.

    def measure(self, true_pos, true_vel):
        pos = true_pos + np.random.normal(0, self.pos_noise_std, 3)
        vel = true_vel + np.random.normal(0, self.vel_noise_std, 3)
        return pos, vel
