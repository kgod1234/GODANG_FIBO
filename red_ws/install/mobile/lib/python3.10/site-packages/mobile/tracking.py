import serial
import json
import cv2
from red_ws.src.abu_red.abu_red.Vision import LineTracking_cam
from mobile.mobile.PID_base import PIDController

class LineTracker:
    def __init__(self, config_path="setting.json"):
        self.config = self.read_config(config_path)
        self.arduino = serial.Serial(
            port=self.config["serial"][0]["port"],
            baudrate=self.config["serial"][1]["baudrate"],
            timeout=self.config["serial"][2]["timeout"]
        )
        self.linetrack_pid = PIDController(kP=1.0, kI=0.1, kD=0.05)
        self.cap = cv2.VideoCapture(1)

    def read_config(self, file_path):
        with open(file_path, "r") as file:
            config = json.load(file)
        return config

    def send_motor_speed(self, x, y, z):
        command = f"{x:.2f};{y:.2f};{z:.2f}\n"
        self.arduino.write(bytes(command, "utf-8"))
        data = self.arduino.readline()
        return data

    def condition_met(self, error):
        return error == 0

    def PID_read(self):
        check, frame = self.cap.read()
        output_BGR2BIN = LineTracking_cam.BGR2BIN(self.cap, threshold=50, height=300, width=300)
        th1 = output_BGR2BIN["th1"]

        output_Horizon = LineTracking_cam.horizontal(100, th1)
        error = output_Horizon["Error"]
        output = self.linetrack_pid.update(error)

        print("PID output:", output, "Error:", error)
        return error

    def run(self):
        while self.cap.isOpened():
            error = self.PID_read()
            if self.condition_met(error):
                break

        self.send_motor_speed(0, 0, 0)  # Stop all motors
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    tracker = LineTracker()
    tracker.run()

