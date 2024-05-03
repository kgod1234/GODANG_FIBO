class PIDController:
    def __init__(self, kP, kI, kD, limit=100):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.limit = limit
        self.P = 0
        self.I = 0
        self.D = 0
        self.previous_error = 0
        self.output = 0

    def update(self, error):
        if (self.output > 0 and error < 0) or (self.output < 0 and error > 0):
            self.I = 0
        if self.P == 0:
            self.I = 0
        
        self.P = error
        self.I += error
        self.D = error - self.previous_error
        
        self.output = round((self.kP * self.P) + (self.kI * self.I) + (self.kD * self.D), 2)
        
        if self.output < -self.limit:
            self.output = -self.limit
        elif self.output > self.limit:
            self.output = self.limit
        
        self.previous_error = error
        
        return self.output