class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_output = min_output
        self.max_output = max_output
        
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        
        self.prev_error = error
        
        output = p_term + i_term + d_term
        # Clamp output to speed limits
        return max(self.min_output, min(output, self.max_output))

    def reset(self):
        """Reset internal history when moving to a new waypoint."""
        self.prev_error = 0.0
        self.integral = 0.0