import numpy as np
import matplotlib.pyplot as plt

class NPOObserver:

    def __init__(self):
        
        self.M  = np.array([
        [124.74,  0.0,  0.0],
        [0.0, 99.1,  -0.525],
        [0.0,  -0.525, 48]
        ])
        self.D  = np.array([
        [2.33, 0.0, 0.0],
        [0.0, 4.67, 7.25],
        [0.0, 0.0, 0.0168]
        ])
        self.Ab = 0
        #self.omega_c = 
        self.Tb = 142.0 * np.eye(3)
        self.L1 = np.diag([4, 4, 4])
        self.L2 = np.diag([9, 9, 7]) * 2
        self.L3 = 0.1 * self.L2


        self.M_inv = np.linalg.inv(self.M)
        self.Tb_inv = np.linalg.inv(self.Tb)

        self.eta_hat = np.zeros(3)
        self.nu_hat  = np.zeros(3)
        self.b_hat   = np.zeros(3)


    def wrap_angle(self, rad: float) -> float:
        return (rad + np.pi) % (2.0 * np.pi) - np.pi
    

    def R(self, psi: float) -> np.ndarray:
        c = np.cos(psi)
        s = np.sin(psi)
        return np.array([
            [ c, -s, 0.0],
            [ s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
    

    def reset(self, eta0, nu0=None, b0=None):
        self.eta_hat = np.array(eta0, dtype=float).reshape(3)
        if nu0 is None:
            self.nu_hat  = np.zeros(3) 
        else: 
           self.nu_hat = np.array(nu0, dtype=float).reshape(3)

        if b0 is None:
            self.b_hat = np.zeros(3)
        else:
            self.b_hat = np.array(b0, dtype=float).reshape(3)

        self.eta_hat[2] = self.wrap_angle(self.eta_hat[2])

    def step(self, dt, eta_meas, tau):
        
        eta_meas = np.array(eta_meas, dtype=float).reshape(3)  # NED
        tau = np.array(tau, dtype=float).reshape(3)            # BODY
    
        eta_tilde = eta_meas - self.eta_hat
        eta_tilde[2] = self.wrap_angle(eta_tilde[2])

        y_tilde = eta_tilde

        psi = float(eta_meas[2])
        Rpsi = self.R(psi)

        eta_hat_dot = Rpsi @ self.nu_hat + self.L1 @ y_tilde

        nu_hat_dot = self.M_inv @ (-self.D @ self.nu_hat + self.b_hat + tau + self.L2 @ (Rpsi.T @ y_tilde))

        b_hat_dot = -self.Tb_inv @ self.b_hat + self.L3 @ (Rpsi.T @ y_tilde)

        self.eta_hat += dt * eta_hat_dot
        self.nu_hat  += dt * nu_hat_dot
        self.b_hat   += dt *  b_hat_dot
        self.eta_hat[2] = self.wrap_angle(self.eta_hat[2])

        return self.eta_hat, self.nu_hat, self.b_hat
