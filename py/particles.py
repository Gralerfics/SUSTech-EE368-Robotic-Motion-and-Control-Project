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


    def predict(self, measurement):
        self.particles += measurement - self.get_average()


    def update(self):
        self.weights = [np.clip(1 / np.linalg.norm(self.get_average() - particle), 0, 100000) for particle in self.particles]
        self.weights /= np.sum(self.weights)


    def resample(self):
        # TODO: 等于没变现在
        indices = np.random.choice(np.arange(self.num_particles), size=self.num_particles, replace=True, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    
    def get_average(self):
        return np.average(self.particles, axis = 0, weights = self.weights)

