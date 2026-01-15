import numpy as np

class WinchMotor:
    def __init__(self, params):
        self.F_max = params["F_winch_max"]

    def get_force(self, t, tangent):
        # simple pulse before release
        if t < 5.0:
            return self.F_max * tangent
        else:
            return np.zeros(2)
