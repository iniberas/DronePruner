"""
vc = [vx, vy, vz, omega]^T  -> kecepatan kamera (translasi x,y,z + yaw rate)
e  = [x - xt, y - yt, z - zt]^T
Ls = interaction matrix 3x4 
vc = -gamma * pinv(Ls) * e
"""

import numpy as np


class IBVSController:
    def __init__(self, fx, fy, cx, cy, gamma=0.5,
                 vmax=1.0, yaw_rate_max=0.6):
        """
        fx, fy, cx, cy : parameter intrinsik kamera (pixel)
        gamma          : gain servo
        vmax           : batas kecepatan translasi (m/s) untuk keamanan
        yaw_rate_max   : batas kecepatan yaw (rad/s)
        """
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.gamma = gamma
        self.vmax = vmax
        self.yaw_rate_max = yaw_rate_max

    def pixel_to_normalized(self, xs, ys):
        x = (xs - self.cx) / self.fx
        y = (ys - self.cy) / self.fy
        return x, y

    def interaction_matrix(self, x, y, z):
        z_safe = z if abs(z) > 1e-6 else 1e-6
        Ls = np.array([
            [-1.0 / z_safe,  0.0,           x / z_safe, -(1.0 + x ** 2)],
            [0.0,           -1.0 / z_safe,  y / z_safe, -x * y],
            [0.0,            0.0,          -1.0,         0.0],
        ])
        return Ls

    def compute_velocity(self, x, y, z, xt, yt, zt):
        """
        Hitung vc = -gamma * Ls^+ * e  (persamaan 4, versi 4-DOF).

        x, y   : koordinat normal target saat ini (persamaan 5)
        z      : jarak kamera ke target (dari depth image)
        xt, yt : koordinat normal target yang diinginkan (persamaan 6)
        zt     : jarak yang diinginkan (mis. jarak ke pisau pemotong)

        Return: np.array([vx, vy, vz, omega])
        """
        e = np.array([x - xt, y - yt, z - zt])
        Ls = self.interaction_matrix(x, y, z)
        Ls_pinv = np.linalg.pinv(Ls)
        vc = -self.gamma * (Ls_pinv @ e)

        vc[0:3] = np.clip(vc[0:3], -self.vmax, self.vmax)
        vc[3] = np.clip(vc[3], -self.yaw_rate_max, self.yaw_rate_max)
        return vc

    @staticmethod
    def camera_to_body_velocity(vc_cam, R_cam_to_body=None):
        vx, vy, vz, omega = vc_cam
        if R_cam_to_body is None:
            R_cam_to_body = np.array([
                [0, 0, 1],
                [1, 0, 0],
                [0, 1, 0],
            ])
        v_body = R_cam_to_body @ np.array([vx, vy, vz])
        return np.array([v_body[0], v_body[1], v_body[2], omega])