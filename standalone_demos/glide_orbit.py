# ============================================================
# GLIDE Orbital Momentum-Exchange Simulation (v3 Enhanced)
# ------------------------------------------------------------
# Based on KenCorp GLIDE Specification v2.3
# Physics: Momentum-exchange tether event between Node + Payload
# Visualization: Corrected animation, zoomed orbits, moving markers
# ============================================================

import numpy as np
import matplotlib
matplotlib.use('QtAgg')   # Use stable modern GUI backend for animation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ------------------------------------------------------------
# PARAMETERS
# ------------------------------------------------------------
μ = 3.986004418e14     # Earth's gravitational parameter [m^3/s^2]
R_E = 6371e3           # Earth radius [m]
h0 = 500e3             # initial altitude [m]
r0 = R_E + h0
v_circ = np.sqrt(μ / r0)

m_node = 25.0          # Node mass [kg]
m_payload = 150.0      # Payload mass [kg]
Δv_payload = 5.0       # Payload delta-v [m/s]
Δv_node = -(m_payload / m_node) * Δv_payload  # momentum conservation

T_event = 3600.0       # event time (1 hour)
dt = 1.0               # timestep
t_final = 8 * 3600.0   # simulate 8 hours
N = int(t_final / dt)

# ------------------------------------------------------------
# INITIAL STATES
# ------------------------------------------------------------
r0_vec = np.array([r0, 0.0])
v0_vec = np.array([0.0, v_circ])
state_node = np.hstack((r0_vec, v0_vec))
state_payload = np.hstack((r0_vec, v0_vec))

# ------------------------------------------------------------
# FUNCTIONS
# ------------------------------------------------------------
def accel(r_vec):
    r = np.linalg.norm(r_vec)
    return -μ * r_vec / r**3

def rk4_step(state, dt):
    r, v = state[:2], state[2:]
    k1_r, k1_v = v, accel(r)
    k2_r, k2_v = v + 0.5*dt*k1_v, accel(r + 0.5*dt*k1_r)
    k3_r, k3_v = v + 0.5*dt*k2_v, accel(r + 0.5*dt*k2_r)
    k4_r, k4_v = v + dt*k3_v, accel(r + dt*k3_r)
    r_next = r + (dt/6)*(k1_r + 2*k2_r + 2*k3_r + k4_r)
    v_next = v + (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)
    return np.hstack((r_next, v_next))

def orbital_elements(state):
    r_vec, v_vec = state[:2], state[2:]
    r, v = np.linalg.norm(r_vec), np.linalg.norm(v_vec)
    h_vec = np.cross(np.append(r_vec,0), np.append(v_vec,0))
    h = np.linalg.norm(h_vec)
    a = 1 / (2/r - v**2/μ)
    ecc_term = max(0.0, 1 - (h**2)/(a*μ))
    e = np.sqrt(ecc_term)
    return r, a, e

def perigee_apogee(a, e):
    rp = a*(1 - e) - R_E
    ra = a*(1 + e) - R_E
    return rp/1e3, ra/1e3  # [km]

# ------------------------------------------------------------
# STORAGE
# ------------------------------------------------------------
t_arr = np.linspace(0, t_final, N)
r_node, r_payload = np.zeros((N,2)), np.zeros((N,2))
a_node, e_node, a_payload, e_payload = [], [], [], []

# ------------------------------------------------------------
# INTEGRATION LOOP
# ------------------------------------------------------------
for i, t in enumerate(t_arr):
    # store
    r_node[i], r_payload[i] = state_node[:2], state_payload[:2]
    rn, an, en = orbital_elements(state_node)
    rp, ap, ep = orbital_elements(state_payload)
    a_node.append(an); e_node.append(en)
    a_payload.append(ap); e_payload.append(ep)

    # tether impulse
    if abs(t - T_event) < dt/2:
        unit_tan = state_payload[2:] / np.linalg.norm(state_payload[2:])
        state_payload[2:] += Δv_payload * unit_tan
        state_node[2:] += Δv_node * unit_tan

    state_node = rk4_step(state_node, dt)
    state_payload = rk4_step(state_payload, dt)

# ------------------------------------------------------------
# SUMMARY
# ------------------------------------------------------------
rp_node, ra_node = perigee_apogee(a_node[-1], e_node[-1])
rp_payload, ra_payload = perigee_apogee(a_payload[-1], e_payload[-1])
print("\n=== GLIDE Orbital Momentum Exchange Summary (v3) ===")
print(f"Initial altitude        : {h0/1e3:.1f} km")
print(f"Tether event            : {T_event/60:.1f} min")
print(f"Δv_payload              : {Δv_payload:.2f} m/s")
print(f"Δv_node                 : {Δv_node:.2f} m/s")
print(f"Node orbit   → Perigee {rp_node:.1f} km | Apogee {ra_node:.1f} km")
print(f"Payload orbit→ Perigee {rp_payload:.1f} km | Apogee {ra_payload:.1f} km")

momentum_before = (m_payload + m_node) * v_circ
momentum_after = m_payload*np.linalg.norm(state_payload[2:]) + m_node*np.linalg.norm(state_node[2:])
print(f"Momentum check          : {momentum_before:.3e} / {momentum_after:.3e}")

# ------------------------------------------------------------
# ANIMATION
# ------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7,7))
ax.set_aspect('equal')
ax.set_xlim(-1.4*r0, 1.4*r0)
ax.set_ylim(-1.4*r0, 1.4*r0)

# Earth + faint orbital guides
earth = plt.Circle((0,0), R_E*0.98, color='blue', alpha=0.5)
ax.add_patch(earth)
ax.plot(r_node[:,0], r_node[:,1], '--', color='orange', alpha=0.3)
ax.plot(r_payload[:,0], r_payload[:,1], '--', color='green', alpha=0.3)

# Moving objects
line_node, = ax.plot([], [], '-', lw=1.2, color='orange', label='Node')
line_payload, = ax.plot([], [], '-', lw=1.2, color='green', label='Payload')
dot_node, = ax.plot([], [], 'o', color='orange')
dot_payload, = ax.plot([], [], 'o', color='green')
event_marker, = ax.plot([], [], 'ro', markersize=8)
ax.legend()

def init():
    for obj in [line_node, line_payload, dot_node, dot_payload, event_marker]:
        obj.set_data([], [])
    return line_node, line_payload, dot_node, dot_payload, event_marker

def update(frame):
    xN, yN = r_node[:frame,0], r_node[:frame,1]
    xP, yP = r_payload[:frame,0], r_payload[:frame,1]

    if len(xN) > 0 and len(xP) > 0:
        line_node.set_data(xN, yN)
        line_payload.set_data(xP, yP)
        dot_node.set_data([xN[-1]], [yN[-1]])
        dot_payload.set_data([xP[-1]], [yP[-1]])

        if T_event - 2 < t_arr[frame] < T_event + 2:
            event_marker.set_data([xP[-1]], [yP[-1]])
        else:
            event_marker.set_data([], [])
    ax.set_title(f"t = {t_arr[frame]/3600:.2f} h — {'Pre-tether' if t_arr[frame]<T_event else 'Post-tether'}")
    return line_node, line_payload, dot_node, dot_payload, event_marker

ani = FuncAnimation(fig, update, frames=N, init_func=init, interval=20, blit=False, repeat=False)
plt.show()

# ------------------------------------------------------------
# DIAGNOSTICS
# ------------------------------------------------------------
fig2, axs = plt.subplots(3,1,figsize=(8,8))
axs[0].plot(t_arr/3600, (np.linalg.norm(r_node,axis=1)-R_E)/1e3, label='Node')
axs[0].plot(t_arr/3600, (np.linalg.norm(r_payload,axis=1)-R_E)/1e3, label='Payload')
axs[0].set_ylabel('Radius [km]'); axs[0].legend()

axs[1].plot(t_arr/3600, np.array(a_node)/1e3, label='Node')
axs[1].plot(t_arr/3600, np.array(a_payload)/1e3, label='Payload')
axs[1].set_ylabel('Semi-major axis [km]'); axs[1].legend()

axs[2].plot(t_arr/3600, e_node, label='Node')
axs[2].plot(t_arr/3600, e_payload, label='Payload')
axs[2].set_xlabel('Time [h]'); axs[2].set_ylabel('Eccentricity'); axs[2].legend()

plt.tight_layout()
plt.show()
