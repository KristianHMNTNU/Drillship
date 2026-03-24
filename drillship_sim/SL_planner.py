import numpy as np


class StraightLinePathPlanner:
    def __init__(self, p0, p1, psi_d, U_ref, mu, eps=1e-3):
        self.p0 = np.array(p0, dtype=float)
        self.p1 = np.array(p1, dtype=float)
        self.psi_d = float(psi_d)

        self.U_ref = float(U_ref)
        self.mu = float(mu)
        self.eps = float(eps)

        self.dp = self.p1 - self.p0
        #self.psi_d = np.arctan2(self.dp[1], self.dp[0])

    def eta_d(self, s):
        p_d = self.p0 + s * self.dp
        return np.array([p_d[0], p_d[1], self.psi_d])

    def eta_d_s(self):
        return np.array([self.dp[0], self.dp[1], 0.0])

    def eta_d_ss(self):
        return np.array([0.0, 0.0, 0.0])

    def speed_profile(self):
        norm_eta_d_s = np.linalg.norm(self.eta_d_s())
        if norm_eta_d_s < 1e-12:
            v_s = 0.0
        else:
            v_s = self.U_ref / norm_eta_d_s

        v_s_s = 0.0
        return v_s, v_s_s

    def V1_s(self, eta_hat, s):
        eta_d = self.eta_d(s)
        eta_d_s = self.eta_d_s()

        psi = eta_hat[2]
        R = np.array([
            [np.cos(psi), -np.sin(psi), 0.0],
            [np.sin(psi),  np.cos(psi), 0.0],
            [0.0,          0.0,         1.0]
        ])

        z1 = R.T @ (eta_hat - eta_d)
        V1_s = - z1.T @ (R.T @ eta_d_s)

        return V1_s

    def s_dot(self, eta_hat, s):
        v_s, _ = self.speed_profile()
        eta_d_s = self.eta_d_s()

        V1_s = self.V1_s(eta_hat, s)
        denom = np.linalg.norm(eta_d_s) + self.eps

        omega_s = - (self.mu / denom) * V1_s
        s_dot = v_s + omega_s

        return s_dot, v_s, V1_s, omega_s

    def get_signals(self, eta_hat, s):
        eta_d = self.eta_d(s)
        eta_d_s = self.eta_d_s()
        eta_d_ss = self.eta_d_ss()

        s_dot, v_s, V1_s, omega_s = self.s_dot(eta_hat, s)
        _, v_s_s = self.speed_profile()

        return eta_d, eta_d_s, eta_d_ss, v_s, v_s_s, s_dot