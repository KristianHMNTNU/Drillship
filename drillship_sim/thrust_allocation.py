import numpy as np


class ThrustAllocator:

    def __init__(self):

        lx1 = 1.068
        ly1 = 0.0
        lx2 = 0.934
        ly2 = -0.11
        lx3 = 0.934
        ly3 = 0.11
        lx4 = -1.164
        ly4 = 0.0
        lx5 = -0.991
        ly5 = -0.164
        lx6 = -0.991
        ly6 = 0.164

        self.B = np.array([      
        [1,       0,       1,       0,       1,       0,      1,       0,       1,       0,       1,       0], 
        [0,       1,       0,       1,       0,      1,      0,       1,       0,       1,       0,       1],
        [-ly1,    lx1,     -ly2,    lx2,     -ly3,    lx3,   -ly4,     lx4,   -ly5,     lx5,   -ly6,     lx6]
        ], dtype='float')

        self.K = np.eye(6)

        self.W = np.eye(12)

        self.f_d = np.zeros(12, dtype='float')
        #self.f_d = np.array([0.0, 0.001, 0.0, -0.001, 0.0], dtype='float')

    def weighted_pinv(self):
        #" B_W^† = W^{-1} B^T (B W^{-1} B^T)^-1 "
        W_inv = np.linalg.inv(self.W)
        return W_inv @ self.B.T @ np.linalg.pinv(self.B @ W_inv @ self.B.T)

    def clip_u(self, u):
        u[0] = np.clip(u[0], 0.0, 1.0)    # azimuth 1
        u[1] = np.clip(u[1], 0.0, 1.0)    # azimuth 2
        u[2] = np.clip(u[2], 0.0, 1.0)    # azimuth 3
        u[3] = np.clip(u[3], 0.0, 1.0)    # azimuth 4
        u[4] = np.clip(u[4], 0.0, 1.0)    # azimuth 5
        u[5] = np.clip(u[5], 0.0, 1.0)    # azimuth 6
        return u


    def wrap_to_pi(self, angle):
        """Wrap angle(s) to [-pi, pi)"""
        return (angle + np.pi) % (2*np.pi) - np.pi


    def allocate(self, tau_cmd):
        B_W_pinv = self.weighted_pinv()
        Q_W = np.eye(12) - B_W_pinv @ self.B

        self.f_star = B_W_pinv @ tau_cmd + Q_W @ self.f_d

        F_1 = np.sqrt(self.f_star[0]**2 + self.f_star[1]**2)
        F_2 = np.sqrt(self.f_star[2]**2 + self.f_star[3]**2)
        F_3 = np.sqrt(self.f_star[4]**2 + self.f_star[5]**2)
        F_4 = np.sqrt(self.f_star[6]**2 + self.f_star[7]**2)
        F_5 = np.sqrt(self.f_star[8]**2 + self.f_star[9]**2)
        F_6 = np.sqrt(self.f_star[10]**2 + self.f_star[11]**2)

        alpha_1 = np.arctan2(self.f_star[1], self.f_star[0])
        alpha_2 = np.arctan2(self.f_star[3], self.f_star[2])
        alpha_3 = np.arctan2(self.f_star[5], self.f_star[4])
        alpha_4 = np.arctan2(self.f_star[7], self.f_star[6])
        alpha_5 = np.arctan2(self.f_star[9], self.f_star[8])
        alpha_6 = np.arctan2(self.f_star[11], self.f_star[10])

        alpha_1 = self.wrap_to_pi(alpha_1)
        alpha_2 = self.wrap_to_pi(alpha_2)
        alpha_3 = self.wrap_to_pi(alpha_3)
        alpha_4 = self.wrap_to_pi(alpha_4)
        alpha_5 = self.wrap_to_pi(alpha_5)
        alpha_6 = self.wrap_to_pi(alpha_6)

        F_cmd = np.array([F_1, F_2, F_3, F_4, F_5, F_6], dtype='float')
        alpha_cmd = np.array([alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6], dtype='float')
        u_cmd = np.linalg.inv(self.K) @ F_cmd

        u_cmd = self.clip_u(u_cmd)
    
        return F_cmd, alpha_cmd, u_cmd
    