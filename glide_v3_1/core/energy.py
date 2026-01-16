# ============================================================
# File: core/energy.py
# GLIDE v3.1 – Energy Tracking System (fixed totals + sane logging)
# ============================================================

from __future__ import annotations
import csv
import numpy as np


class EnergyTracker:
    """Tracks major energy buckets + battery SoC.

    Fixes:
      • Kinetic energy uses SUM, not mean.
      • Gravity computed consistently for local vs orbital.
      • Logging interval uses a real timer.
    """

    def __init__(self, battery_capacity_J: float = 5e5, log_interval_s: float = 1.0):
        self.time = []
        self.E_kin = []
        self.E_elastic = []
        self.E_grav = []
        self.E_batt = []
        self.E_total = []
        self.SoC = []

        self.E_batt_capacity = float(battery_capacity_J)
        self.E_batt_now = float(battery_capacity_J)

        self.log_interval_s = max(0.0, float(log_interval_s))
        self._t_last_print = -1e9

    def update(self, dt: float, tether, motor, edt, gravity=None):
        dt = float(dt)
        t_next = 0.0 if not self.time else self.time[-1] + dt

        v2 = np.sum(tether.velocities[1:] ** 2, axis=1)
        kinetic = 0.5 * tether.m_segment * float(np.sum(v2))

        elastic = 0.0
        for i in range(tether.N):
            p0, p1 = tether.positions[i], tether.positions[i + 1]
            dist = float(np.linalg.norm(p1 - p0))
            stretch = dist - tether.segment_length
            elastic += 0.5 * tether.k * (stretch * stretch)

        grav = 0.0
        if gravity is not None:
            if getattr(gravity, "mode", "local") == "local":
                y_ref = float(tether.positions[0, 1])
                for i in range(1, tether.N + 1):
                    grav += tether.m_segment * gravity.g0 * (float(tether.positions[i, 1]) - y_ref)
            else:
                U_ref = float(gravity.get_potential(tether.positions[0]))
                for i in range(1, tether.N + 1):
                    Ui = float(gravity.get_potential(tether.positions[i]))
                    grav += tether.m_segment * (Ui - U_ref)

        P_motor = float(getattr(motor, "P_elec", 0.0))
        P_edt = float(edt.get_power())
        P_total = P_motor + P_edt

        dE_batt = -P_total * dt
        self.E_batt_now = float(np.clip(self.E_batt_now + dE_batt, 0.0, self.E_batt_capacity))

        E_total = kinetic + elastic + grav + self.E_batt_now

        self.time.append(t_next)
        self.E_kin.append(kinetic)
        self.E_elastic.append(elastic)
        self.E_grav.append(grav)
        self.E_batt.append(self.E_batt_now)
        self.E_total.append(E_total)
        self.SoC.append(self.E_batt_now / self.E_batt_capacity if self.E_batt_capacity > 0 else 0.0)

        if self.log_interval_s > 0 and (t_next - self._t_last_print) >= self.log_interval_s:
            self._t_last_print = t_next
            print(
                f"[Energy] t={t_next:7.2f}s | SoC={self.SoC[-1]*100:7.3f}% | "
                f"P_motor={P_motor:9.2f}W | P_edt={P_edt:9.2f}W | "
                f"E_total={E_total:12.2f}J | E_k={kinetic:10.2f} | E_el={elastic:10.2f} | E_g={grav:10.2f}"
            )

    def summary(self):
        if not self.time:
            return {}
        return {
            "E_kin": self.E_kin[-1],
            "E_elastic": self.E_elastic[-1],
            "E_grav": self.E_grav[-1],
            "E_batt": self.E_batt[-1],
            "E_total": self.E_total[-1],
            "SoC": self.SoC[-1],
        }

    def export_csv(self, filename: str = "glide_v3_energy_log.csv"):
        with open(filename, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time", "E_kin", "E_elastic", "E_grav", "E_batt", "E_total", "SoC"])
            for i in range(len(self.time)):
                w.writerow([
                    self.time[i],
                    self.E_kin[i],
                    self.E_elastic[i],
                    self.E_grav[i],
                    self.E_batt[i],
                    self.E_total[i],
                    self.SoC[i],
                ])
