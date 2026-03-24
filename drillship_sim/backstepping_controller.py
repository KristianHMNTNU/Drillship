import numpy as np


class backstepping_controller:

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

        self.K1 = np.diag([1.5, 1.5, 2.0])
        self.K2 = np.diag([3.5, 3.5, 4.0])
        #self.K1 = np.diag([1.0, 1.0, 1.0])
        #self.K2 = np.diag([5.0, 5.0, 2.0])


        self.S = np.array([
                [0, -1, 0],
                [1,  0, 0],
                [0,  0, 0]
                ])

    def R(self, psi):
        return np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [0, 0, 1]
        ])


    def step(self, eta_hat, nu_hat, b_hat, eta_d, eta_d_s, eta_d_ss, v_s, v_s_s, s_dot):

        psi = eta_hat[2]
        r = nu_hat[2]
        R = self.R(psi)

        z1 = R.T @ (eta_hat - eta_d)

        alpha1 = -self.K1 @ z1 + R.T @ eta_d_s * v_s

        z2 = nu_hat - alpha1

        z1_dot = nu_hat - r * self.S @ z1 - R.T @ eta_d_s * s_dot

        alpha1_dot = (
            -self.K1 @ z1_dot
            - r * self.S @ (R.T @ eta_d_s * v_s)
            + R.T @ eta_d_ss * v_s * s_dot
            + R.T @ eta_d_s * v_s_s * s_dot
        )

        # Control law
        tau = self.D @ nu_hat - b_hat + self.M @ alpha1_dot - self.K2 @ z2
        return tau