class BeamSteering:
    def __init__(self, min_elev=10.0, hysteresis=4.0):
        self.min_elev = min_elev
        self.hysteresis = hysteresis
        self.current_id = None

    def select(self, sats):
        """
        Select the best satellite and return:
        - sat_id
        - desired azimuth (deg)
        - desired elevation (deg)
        """
        visible = [s for s in sats if s['elev'] > self.min_elev]
        
        if not visible:
            return None, None, None

        best = max(visible, key=lambda s: s['quality'])

        # Hand-off with hysteresis
        if self.current_id is None:
            self.current_id = best['id']
        else:
            current_quality = next(
                (s['quality'] for s in sats if s['id'] == self.current_id), 
                0.0
            )
            if best['quality'] > current_quality + self.hysteresis:
                self.current_id = best['id']

        target = next(s for s in sats if s['id'] == self.current_id)
        
        return self.current_id, target['az'], target['elev']

