# import numpy as np


# class KalmanFilter:
#     def __init__(self, dt, state_dim, measurement_dim):
#         self.dt = dt
#         self.A = np.eye(state_dim)          # 状态转移矩阵
#         self.B = np.eye(state_dim)          # 控制输入矩阵
#         self.H = np.eye(measurement_dim)    # 测量矩阵

#         # 过程噪声和测量噪声协方差矩阵
#         self.Q = np.eye(state_dim)          # 过程噪声协方差矩阵
#         self.R = np.eye(measurement_dim)    # 测量噪声协方差矩阵

#         # 状态估计
#         self.x = np.zeros((state_dim, 1))   # 初始状态估计
#         self.P = np.eye(state_dim)          # 初始状态协方差矩阵

#     def predict(self, u = np.array([[0], [0], [0], [0], [0], [0]])):
#         self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
#         self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

#     def update(self, z):
#         y = z - np.dot(self.H, self.x)
#         S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
#         K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

#         self.x = self.x + np.dot(K, y)
#         self.P = np.dot((np.eye(self.P.shape[0]) - np.dot(K, self.H)), self.P)

