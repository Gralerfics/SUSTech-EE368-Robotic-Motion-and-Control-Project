import numpy as np
import time


class PIDController:
    def __init__(self, Kp, Ki, Kd, dim):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dim = dim
    
    
    def start(self):
        self.E_acc = np.zeros(self.dim)
        self.last_time = time.time()
        self.last_error = np.zeros(self.dim)
    
    
    def step(self, error, error_dot):
        dt = time.time() - self.last_time
        self.last_time = time.time()
        
        P = self.Kp * error
        
        self.E_acc += error * dt
        I = self.Ki * self.E_acc
        
        if error_dot is None:
            derror = (error - self.last_error)
            self.last_error = error
            D = self.Kd * derror / dt
        else:
            D = self.Kd * error_dot
        
        return P + I + D

