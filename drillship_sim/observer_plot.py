import numpy as np
import matplotlib.pyplot as plt

from NPO import NPOObserver

dt = 0.01
T  = 200.0
N  = int(T/dt) + 1
t  = np.linspace(0.0, T, N)

noise_power = 1e-5
sigma = np.sqrt(noise_power)

obs = NPOObserver()

eta_tilde = np.array([1.0, 1.0, np.pi/4])
nu_tilde  = np.array([0.1, 0.1, 0.0])
b_tilde   = np.array([0.1, -0.1, 0.01])

tau = np.zeros(3)

eta_tilde_log = np.zeros((N,3))
nu_tilde_log  = np.zeros((N,3))
b_tilde_log   = np.zeros((N,3))


for k, tk in enumerate(t):

    psi = obs.wrap_angle(0.1 * tk)
    w = sigma * np.random.randn(3)

    y_tilde = eta_tilde + w
    y_tilde[2] = obs.wrap_angle(y_tilde[2])

    Rpsi = obs.R(psi)

    eta_tilde_dot = Rpsi @ nu_tilde - obs.L1 @ y_tilde
    nu_tilde_dot  = obs.M_inv @ (-obs.D @ nu_tilde + b_tilde - obs.L2 @ (Rpsi.T @ y_tilde))
    b_tilde_dot   = -obs.Tb_inv @ b_tilde - obs.L3 @ (Rpsi.T @ y_tilde)

    eta_tilde += dt * eta_tilde_dot
    nu_tilde  += dt * nu_tilde_dot
    b_tilde   += dt * b_tilde_dot
    eta_tilde[2] = obs.wrap_angle(eta_tilde[2])

    eta_tilde_log[k,:] = eta_tilde
    nu_tilde_log[k,:]  = nu_tilde
    b_tilde_log[k,:]   = b_tilde


plt.figure(figsize=(8,5))
plt.plot(t, eta_tilde_log[:,0], label=r'$\tilde{x}$')
plt.plot(t, eta_tilde_log[:,1], label=r'$\tilde{y}$')
plt.plot(t, eta_tilde_log[:,2], label=r'$\tilde{\psi}$')
plt.xlabel('Time [s]')
plt.ylabel(r'$\tilde{\eta}$')
plt.grid(True)
plt.legend()
plt.tight_layout()
#plt.savefig("C:\\Users\\krist\\Desktop\\NTNU\\8. Semester\\Marreg\\Lab prosjekt\\Case B\\Plots\\eta_tilde.png", dpi=300)


plt.figure(figsize=(8,5))
plt.plot(t, nu_tilde_log[:,0], label=r'$\tilde{u}$')
plt.plot(t, nu_tilde_log[:,1], label=r'$\tilde{v}$')
plt.plot(t, nu_tilde_log[:,2], label=r'$\tilde{r}$')
plt.xlabel('Time [s]')
plt.ylabel(r'$\tilde{\nu}$')
plt.grid(True)
plt.legend()
plt.tight_layout()
#plt.savefig("C:\\Users\\krist\\Desktop\\NTNU\\8. Semester\\Marreg\\Lab prosjekt\\Case B\\Plots\\nu_tilde.png", dpi=300)


plt.figure(figsize=(8,5))
plt.plot(t, b_tilde_log[:,0], label=r'$\tilde{b}_x$')
plt.plot(t, b_tilde_log[:,1], label=r'$\tilde{b}_y$')
plt.plot(t, b_tilde_log[:,2], label=r'$\tilde{b}_\psi$')
plt.xlabel('Time [s]')
plt.ylabel(r'$\tilde{b}$')
plt.grid(True)
plt.legend()
plt.tight_layout()
#plt.savefig("C:\\Users\\krist\\Desktop\\NTNU\\8. Semester\\Marreg\\Lab prosjekt\\Case B\\Plots\\b_tilde.png", dpi=300)

plt.show()