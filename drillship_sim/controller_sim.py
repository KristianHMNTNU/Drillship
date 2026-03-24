import numpy as np
import matplotlib.pyplot as plt

from backstepping_controller import backstepping_controller
from SL_planner import StraightLinePathPlanner


# =========================
# Simulation settings
# =========================
dt = 0.01
T = 1000
N = int(T / dt)

# =========================
# Initial conditions
# =========================
eta = np.array([-10.0, 10.0, 0.0])   # [x, y, psi]
nu = np.array([0.0, 0.0, 0.0])
b = np.zeros(3)

s = 0.0

def wrap_to_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


# =========================
# Instantiate system
# =========================
planner = StraightLinePathPlanner(
    p0=[0, 0],
    p1=[20, 40],
    psi_d=np.deg2rad(90),
    U_ref=0.3,
    mu=0.001
)

controller = backstepping_controller()


# =========================
# Logging
# =========================
eta_log = np.zeros((N, 3))
eta_d_log = np.zeros((N, 3))
nu_log = np.zeros((N, 3))
tau_log = np.zeros((N, 3))
s_log = np.zeros(N)
z1_log = np.zeros((N, 3))
z2_log = np.zeros((N, 3))


# =========================
# Simulation loop
# =========================
for i in range(N):

    # Planner
    eta_d, eta_d_s, eta_d_ss, v_s, v_s_s, s_dot = planner.get_signals(eta, s)

    if s >= 1.0:
        s_dot = 0.0
        v_s = 0.0
        #v_s_s = 0.0

    # Controller (ALLTID aktiv)
    tau = controller.step(
        eta, nu, b,
        eta_d, eta_d_s, eta_d_ss,
        v_s, v_s_s, s_dot
    )

    # Rotation
    psi = eta[2]
    R = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi),  np.cos(psi), 0],
        [0, 0, 1]
    ])

    # Error (kun for logging)
    z1 = R.T @ (eta - eta_d)
    z1[2] = wrap_to_pi(eta[2] - eta_d[2])

    alpha1 = -controller.K1 @ z1 + (R.T @ eta_d_s) * v_s
    z2 = nu - alpha1

    z1_log[i] = z1
    z2_log[i] = z2

    # Dynamics
    nu_dot = np.linalg.inv(controller.M) @ (tau - controller.D @ nu + b)
    eta_dot = R @ nu

    # Integrate
    nu = nu + dt * nu_dot
    eta = eta + dt * eta_dot
    s = s + dt * s_dot
    s = np.clip(s, 0.0, 1.0)

    # Log
    eta_log[i] = eta
    eta_d_log[i] = eta_d
    nu_log[i] = nu
    tau_log[i] = tau
    s_log[i] = s


# =========================
# Plot position
# =========================
plt.figure(figsize=(8,6))

p0 = planner.p0
p1 = planner.p1

plt.plot([p0[0], p1[0]], [p0[1], p1[1]], 'k--', label='Path')
plt.plot(p0[0], p0[1], "ko")
plt.plot(p1[0], p1[1], "ko")

plt.plot(eta_log[:,0], eta_log[:,1], label="Vessel")
plt.scatter(eta_log[0,0], eta_log[0,1], c='green', s=80, label="Start")
plt.scatter(eta_log[-1,0], eta_log[-1,1], c='red', s=80, label="End")

# Heading arrows
skip = 1000
for i in range(0, N, skip):
    x = eta_log[i,0]
    y = eta_log[i,1]
    psi = eta_log[i,2]

    dx = np.cos(psi)
    dy = np.sin(psi)

    plt.arrow(x, y, dx, dy,
              head_width=0.3,
              head_length=0.5,
              fc='blue', ec='blue')

# Reference heading
psi_d = planner.psi_d
x_mid = (p0[0] + p1[0]) / 2
y_mid = (p0[1] + p1[1]) / 2

plt.arrow(x_mid, y_mid,
          np.cos(psi_d), np.sin(psi_d),
          head_width=0.4, head_length=0.7,
          fc='black', ec='black',
          label="Reference heading")

plt.xlabel("x")
plt.ylabel("y")
plt.axis("equal")
plt.grid()
plt.legend()
plt.title("Path Following with Heading")
plt.show()


# =========================
# Error plots
# =========================
t = np.linspace(0, T, N)

plt.figure()

plt.subplot(3,1,1)
plt.plot(t, z1_log[:,0], label="z1_x")
plt.plot(t, z2_log[:,0], label="z2_u")
plt.legend()
plt.grid()

plt.subplot(3,1,2)
plt.plot(t, z1_log[:,1], label="z1_y")
plt.plot(t, z2_log[:,1], label="z2_v")
plt.legend()
plt.grid()

plt.subplot(3,1,3)
plt.plot(t, z1_log[:,2], label="z1_psi")
plt.plot(t, z2_log[:,2], label="z2_r")
plt.legend()
plt.grid()

plt.xlabel("Time [s]")
plt.suptitle("Error components")
plt.show()