#!/usr/bin/env python3
"""
GNC Simulator - Clean Entry Point
Only imports and orchestrates the modules.
"""

from simulation.simulator import run_simulation

if __name__ == "__main__":
    # You can easily change parameters here
    results = run_simulation(
        duration=90.0,      # seconds
        dt=0.05,            # 20 Hz
        plot=True,
        save_plots=True
    )
