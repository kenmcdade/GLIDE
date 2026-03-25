"""Tag mode state machine (range-based)."""


class TagModeManager:
    def __init__(self, thresholds):
        self.thresholds = thresholds
        self.mode = "COARSE"

    def update(self, range_m):
        if range_m <= self.thresholds["R_PRE"]:
            self.mode = "TERMINAL"
        elif range_m <= self.thresholds["R_COR"]:
            self.mode = "TERMINAL_PREP"
        elif range_m <= self.thresholds["R_ACQ"]:
            self.mode = "REFINE"
        else:
            self.mode = "COARSE"
        return self.mode
