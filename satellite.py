import numpy as np

class SatelliteConstellation:
    def __init__(self, num_sats=6):
        self.num_sats = num_sats
        # Simplified circular orbits (not real sat)
        self.radii = 550e3 + 6371e3  # ~550 km altitude
        self.inclinations = np.linspace(50, 60, num_sats) * np.pi/180
        self.phases = np.linspace(0, 2*np.pi, num_sats, endpoint=False)

    def get_positions(self, time, observer_pos):
        """Return satellite positions in ECEF-like frame and elevation/azimuth from observer."""
        sats = []
        for i in range(self.num_sats):
            # Very simplified Keplerian motion
            theta = self.phases[i] + 0.001 * time  # angular rate approx
            x = self.radii * np.cos(theta)
            y = self.radii * np.sin(theta) * np.cos(self.inclinations[i])
            z = self.radii * np.sin(theta) * np.sin(self.inclinations[i])
            sat_pos = np.array([x, y, z])
            
            # Relative vector
            rel = sat_pos - observer_pos
            range_ = np.linalg.norm(rel)
            elev = np.arcsin(rel[2] / range_) * 180/np.pi   # crude elevation
            az = np.arctan2(rel[1], rel[0]) * 180/np.pi
            sats.append({
                'id': i,
                'pos': sat_pos,
                'elev': elev,
                'az': az,
                'range': range_,
                'quality': max(0, elev)  # simple link integrity proxy (higher elev better)
            })
        return sats
