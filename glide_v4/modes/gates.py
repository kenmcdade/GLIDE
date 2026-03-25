"""Corridor gate tracking and speed checks."""

class GateTracker:
    def __init__(self, thresholds, speed_limits):
        self.thresholds = thresholds
        self.speed_limits = speed_limits
        self.crossings = {}
        self.violations = []

    def update(self, range_m, speed_mps, t, prev_range=None, prev_speed=None):
        events = []
        for name, thr in self.thresholds.items():
            if name in self.crossings:
                continue
            crossed = range_m <= thr
            if prev_range is not None and prev_range > thr and range_m <= thr:
                # Linear interpolation of crossing time/speed.
                frac = (prev_range - thr) / max(prev_range - range_m, 1e-9)
                cross_speed = float(prev_speed + frac * (speed_mps - prev_speed))
                cross_range = float(thr)
            elif crossed:
                cross_speed = float(speed_mps)
                cross_range = float(range_m)
            else:
                continue

            if crossed:
                self.crossings[name] = {
                    "t": float(t),
                    "range_m": float(cross_range),
                    "speed_mps": float(cross_speed),
                }
                events.append(name)
                limits = self.speed_limits.get(name, {})
                hard_max = limits.get("hard_max")
                if hard_max is not None and cross_speed > hard_max:
                    self.violations.append({
                        "gate": name,
                        "type": "speed_violation",
                        "speed_mps": float(cross_speed),
                        "hard_max": float(hard_max),
                        "t": float(t),
                    })
        return events
