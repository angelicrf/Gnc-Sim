import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from pathlib import Path

from models.platform_sim import MovingPlatform
from models.sensors import IMU, GNSS
from models.satellite import SatelliteConstellation
from control.beam_steering import BeamSteering
from estimation.ekf_attitude import AttitudeINS_EKF

def run_simulation(duration=90.0, dt=0.05, plot=True, save_plots=True):
    print("=" * 60)
    print("GNC Simulator")
    print("=" * 60)

    # Initialize all modules
    platform_sim = MovingPlatform(dt=dt)
    imu = IMU()
    gnss = GNSS()
    constellation = SatelliteConstellation(num_sats=8)
    beam = BeamSteering(min_elev=10.0)
    ekf = AttitudeINS_EKF(dt=dt)

    # Logs
    times, true_pos_log, est_pos_log = [], [], []
    sat_ids, elevations, pointing_errors, qualities = [], [], [], []

    steps = int(duration / dt)
    print(f"Running {steps} steps ({duration}s) @ {1/dt:.0f} Hz...")

    for i in range(steps):
        t = i * dt

        # True dynamics (gentle ship motion)
        true_omega = np.array([
            0.02 * np.sin(0.3 * t),
            0.015 * np.cos(0.25 * t),
            0.005 * np.sin(0.1 * t)
        ])
        true_accel = np.array([0.0, 0.0, -9.81])

        true_pos, true_vel, true_quat = platform_sim.update(true_accel, true_omega)

        # Sensors
        gyro_meas, accel_meas = imu.measure(true_omega, true_accel)
        gnss_pos, gnss_vel = gnss.measure(true_pos, true_vel)

        # Estimation
        ekf.predict(gyro_meas, accel_meas)
        est_pos, est_vel, est_quat = ekf.update(gnss_pos, gnss_vel)

        # Beam steering
        sats = constellation.get_positions(t, true_pos)
        sat_id, des_az, des_el = beam.select(sats)

        # Simple pointing error (placeholder)
        pointing_error = 0.4 + 0.3 * np.sin(0.2 * t) + np.random.normal(0, 0.12) if sat_id is not None else 8.0

        # Logging
        times.append(t)
        true_pos_log.append(true_pos)
        est_pos_log.append(est_pos)
        sat_ids.append(sat_id if sat_id is not None else -1)
        elevations.append(des_el if des_el is not None else 0.0)
        qualities.append(next((s['quality'] for s in sats if s['id'] == sat_id), 0) if sat_id else 0)
        pointing_errors.append(pointing_error)

        if i % 200 == 0:
            print(f"  t={t:6.1f}s | Sat={sat_id} | Elev={des_el:5.1f}° | Err={pointing_error:4.2f}°")

    # Convert to arrays
    times = np.array(times)
    true_pos_log = np.array(true_pos_log)
    est_pos_log = np.array(est_pos_log)
    sat_ids = np.array(sat_ids)
    elevations = np.array(elevations)
    pointing_errors = np.array(pointing_errors)
    qualities = np.array(qualities)
    pos_error = np.linalg.norm(true_pos_log - est_pos_log, axis=1)

    # ==================== Visualization ====================
    if plot:
        fig = plt.figure(figsize=(16, 11))
        gs = GridSpec(3, 2, figure=fig)

        # Trajectory
        ax1 = fig.add_subplot(gs[0, 0])
        ax1.plot(true_pos_log[:, 1], true_pos_log[:, 0], 'b-', label='True', lw=2)
        ax1.plot(est_pos_log[:, 1], est_pos_log[:, 0], 'r--', label='Estimated', alpha=0.8)
        ax1.set_xlabel('East (m)')
        ax1.set_ylabel('North (m)')
        ax1.set_title('Platform Trajectory')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

        # Position error
        ax2 = fig.add_subplot(gs[0, 1])
        ax2.plot(times, pos_error, 'g-')
        ax2.set_title('Position Estimation Error')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Error (m)')
        ax2.grid(True)

        # Satellite hand-offs
        ax3 = fig.add_subplot(gs[1, 0])
        ax3.plot(times, sat_ids, 'o-', markersize=3)
        ax3.set_title('Satellite Tracking & Hand-offs')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Satellite ID')
        ax3.grid(True)

        # Elevation
        ax4 = fig.add_subplot(gs[1, 1])
        ax4.plot(times, elevations, 'm-')
        ax4.axhline(10, color='r', ls='--', label='Min Elev')
        ax4.set_title('Tracked Satellite Elevation')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Elevation (°)')
        ax4.legend()
        ax4.grid(True)

        # Pointing error
        ax5 = fig.add_subplot(gs[2, 0])
        ax5.plot(times, pointing_errors, 'c-')
        ax5.set_title('Antenna Pointing Error')
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Error (°)')
        ax5.grid(True)

        # Link quality
        ax6 = fig.add_subplot(gs[2, 1])
        ax6.plot(times, qualities, color='orange')
        ax6.set_title('Link Integrity Metric')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Quality')
        ax6.grid(True)

        plt.tight_layout()

        if save_plots:
            Path("results").mkdir(exist_ok=True)
            fig.savefig("results/gnc_simulation_results.png", dpi=150, bbox_inches='tight')
            print("Plot saved → results/gnc_simulation_results.png")

        plt.show()

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print(f"Avg position error : {np.mean(pos_error):.2f} m")
    print(f"Max position error : {np.max(pos_error):.2f} m")
    print(f"Avg pointing error : {np.mean(pointing_errors):.2f}°")
    print(f"Number of hand-offs: {np.sum(np.diff(sat_ids) != 0)}")
    print("=" * 60)

    return {
        "times": times,
        "true_pos": true_pos_log,
        "est_pos": est_pos_log,
        "sat_ids": sat_ids,
        "elevations": elevations,
        "pointing_errors": pointing_errors
    }
