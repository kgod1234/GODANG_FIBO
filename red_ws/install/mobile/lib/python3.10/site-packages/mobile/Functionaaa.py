import math
from PID import PIDController

class PositionController:
    def __init__(self):
        self.x, self.y, self.theta = 0, 0, 0
        self.pos_x = PIDController(kP=0.1, kI=0.05, kD=0)
        self.pos_y = PIDController(kP=0.1, kI=0.05, kD=0)
        self.pos_z = PIDController(kP=0.01, kI=0.001, kD=0)
        self.vx, self.vy, self.vz = 0, 0, 0
        self.reset = 0;
        
    def position_reset(self):
        self.reset = 1;

    def angular_difference(self, target, current):
        difference = target - current
        if difference < -180:
            difference += 360
        if difference > 180:
            difference -= 360
        return difference

    def go_to_position(self, target_x, target_y, target_z, pos_x, pos_y, pos_z):
        
        if self.reset == 1:
            pass
        else:
            error_x = target_x - pos_x
            error_y = target_y - pos_y
            error_z = self.angular_difference(target_z, pos_z)

            error_magnitude = math.sqrt(error_x**2 + error_y**2)
            error_direction = math.atan2(error_y, error_x)

            vx = self.pos_x.update(error_magnitude * math.cos(error_direction))
            vy = self.pos_y.update(error_magnitude * math.sin(error_direction))
            vz = self.pos_z.update(error_z)

            if abs(error_x) <= 0.05 and abs(error_y) <= 0.05 and abs(error_z) <= 3:
                return [0,0,0]
            else:
                return [vx,vy,vz]