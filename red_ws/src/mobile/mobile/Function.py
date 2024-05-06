import cv2
import time
import math

#PID
import PID

PosX = PID.PIDController(kP=0.1, kI=0.05, kD=0)
PosY = PID.PIDController(kP=0.1, kI=0.05, kD=0)
PosZ = PID.PIDController(kP=0.01, kI=0.001, kD=0)

def SendVelocity(x, y, z):
    command = f"{x:.2f};{y:.2f};{z:.2f}\n"
    arduino.write(bytes(command, "utf-8"))
    data = arduino.readline()
    return data

def PositionReset():
    arduino.write(b'Reset\n')
    data = arduino.readline()
    return data

def ReadPosition():
    position_data = arduino.readline().decode().strip()
    if position_data:
        try:
            x, y, theta = map(float, position_data.split(','))
            return x, y, theta
        except ValueError:
            return None
    else:
        return None

def angular_difference(target, current):
    difference = target - current
    while difference < -180:
        difference += 360
    while difference > 180:
        difference -= 360
    return difference

def GoToPosition(target_x, target_y, target_z):
    
    PositionReset()

    while True:
        current_position = ReadPosition()
        if current_position is None:
            continue

        current_x, current_y, current_z = current_position
        error_x = target_x - current_x
        error_y = target_y - current_y
        error_z = angular_difference(target_z, current_z)

        error_magnitude = math.sqrt(error_x**2 + error_y**2)
        error_direction = math.atan2(error_y, error_x)

        vx = PosX.update(error_magnitude * math.cos(error_direction))
        vy = PosY.update(error_magnitude * math.sin(error_direction))
        vz = PosZ.update(error_z)

        SendVelocity(vx, vy, vz)

        if abs(error_x) <= 0.03 and abs(error_y) <= 0.03 and abs(error_z) <= 1:
            SendVelocity(0, 0, 0)
            break

        # print(f"Moving to ({target_x}, {target_y}, {target_z}) from ({current_x}, {current_y}, {current_z})")
        # time.sleep(100)
