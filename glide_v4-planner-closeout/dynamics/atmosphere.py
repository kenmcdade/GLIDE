"""Atmosphere density models (simple stubs)."""

import numpy as np


class AtmosphereModel:
    def __init__(self, model_type, params):
        self.model_type = model_type
        self.params = params
        if model_type == "table":
            alts_km = np.array(params["alt_km"], dtype=float)
            rhos = np.array(params["rho"], dtype=float)
            self._alt_km = alts_km
            self._log_rho = np.log(np.maximum(rhos, 1e-30))

    def density(self, alt_m):
        alt_m = max(0.0, float(alt_m))
        rho_scale = float(self.params.get("rho_scale", 1.0))
        if self.model_type == "exponential":
            rho0 = float(self.params["rho0"])
            h0 = float(self.params["h0"])
            H = float(self.params["H"])
            return rho_scale * rho0 * np.exp(-(alt_m - h0) / H)
        if self.model_type == "table":
            alt_km = alt_m / 1000.0
            log_rho = np.interp(alt_km, self._alt_km, self._log_rho)
            return float(rho_scale * np.exp(log_rho))
        raise ValueError(f"Unknown atmosphere model: {self.model_type}")
