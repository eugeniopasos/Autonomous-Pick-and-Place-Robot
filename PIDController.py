import numpy as np
class PIDController:
    def __init__(self, dim=3, dt=0.02):
        # Initialize gains (tuned for position control in mm)
        self.Kp = 0.3 * np.eye(dim) # Proportional gain
        self.Ki = 0.003* np.eye(dim) # Integral gain
        self.Kd = 0.01* np.eye(dim) # Derivative gain
        # Initialize error terms
        self.error_integral = np.zeros(dim)
        self.error_prev = np.zeros(dim)
        self.dt = dt # Control period in seconds
    def compute_pid(self, error):
        P = self.Kp @ error
        self.error_integral += error*self.dt
        I = self.Ki @ self.error_integral
        D = self.Kd @ ((error - self.error_prev)/self.dt)
        self.error_prev = error
        return P + I + D
