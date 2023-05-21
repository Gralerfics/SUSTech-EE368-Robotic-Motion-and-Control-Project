import numpy as np

class ParticleFilter:
    def __init__(self, num_particles, state_dim, measurement_dim):
        self.num_particles = num_particles
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        self.particles = np.zeros((num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles

    def initialize_particles(self, initial_state):
        self.particles = np.tile(initial_state, (self.num_particles, 1))

    def predict(self, motion_model):
        self.particles = motion_model(self.particles)

    def update(self, measurement, measurement_model):
        self.weights = measurement_model(measurement, self.particles)
        self.weights /= np.sum(self.weights)  # 归一化权重

    def resample(self):
        indices = np.random.choice(np.arange(self.num_particles), size=self.num_particles, replace=True, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

# 使用示例
num_particles = 1000  # 粒子数量
state_dim = 6  # 状态维度
measurement_dim = 6  # 测量维度

# 创建粒子滤波器
pf = ParticleFilter(num_particles, state_dim, measurement_dim)

while True:
    # 获取ArUco位姿测量值 (rvec, tvec)
    rvec, tvec = get_aruco_pose_measurement()

    # 将旋转向量和平移向量组合成位姿向量
    pose_measurement = np.concatenate((rvec, tvec), axis=0)

    # 预测步骤
    pf.predict(motion_model)

    # 更新步骤
    pf.update(pose_measurement, measurement_model)

    # 重采样步骤
    pf.resample()

    # 获取滤波后的位姿估计
    pose_estimate = np.average(pf.particles, axis=0, weights=pf.weights)

