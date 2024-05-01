#include "Arduino.h"
#include "Kinematics.h"

Kinematics::Kinematics(float wheel_diameter, float lx, float ly) {
  robot.wheel_diameter = wheel_diameter;
  robot.lx = lx;
  robot.ly = ly;
  robot.wheel_circumference = robot.wheel_diameter * M_PI;
  robot.angular_to_rpm = 60 / robot.wheel_circumference;
}

Kinematics::Velocity Kinematics::Forward_Kinematics_Velocity(float RPM_FL, float RPM_FR, float RPM_BL, float RPM_BR) {
  Velocity basev;
  basev.vx = ((robot.wheel_diameter / 2.0) / 4.0) * (RPM_FL + RPM_FR + RPM_BL + RPM_BR) * 0.1047198;
  basev.vy = ((robot.wheel_diameter / 2.0) / 4.0) * (-RPM_FL + RPM_FR + RPM_BL - RPM_BR) * 0.1047198;
  basev.wz = ((robot.wheel_diameter / 2.0) / (4.0 * (robot.lx + robot.ly))) * (-RPM_FL + RPM_FR - RPM_BL + RPM_BR) * 0.1047198;
  return basev;
}

Kinematics::RPM Kinematics::Inverse_Kinematics(float vx, float vy, float wz) {
  RPM wheel_rpm;
  if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
    wheel_rpm.RPM_FL = 0;
    wheel_rpm.RPM_FR = 0;
    wheel_rpm.RPM_BL = 0;
    wheel_rpm.RPM_BR = 0;
  } else {
    wheel_rpm.RPM_FL = (vx - vy - (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5) * 9.5492968;
    wheel_rpm.RPM_FR = (vx + vy + (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5) * 9.5492968;
    wheel_rpm.RPM_BL = (vx + vy - (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5) * 9.5492968;
    wheel_rpm.RPM_BR = (vx - vy + (robot.lx + robot.ly) * wz) / (robot.wheel_diameter * 0.5) * 9.5492968;
  }
  return wheel_rpm;
}

Kinematics::Position Kinematics::Forward_Kinematics_Position(float RPM_FL, float RPM_FR, float RPM_BL, float RPM_BR, Position current_position, float deltaTime) {
  Velocity basev = Forward_Kinematics_Velocity(RPM_FL, RPM_FR, RPM_BL, RPM_BR);
  current_position.x += basev.vx * deltaTime;
  current_position.y += basev.vy * deltaTime;
  current_position.theta += basev.wz * deltaTime * (180.0 / M_PI);
  current_position.theta = fmod(current_position.theta, 360.0);

  if (current_position.theta < 0) {
    current_position.theta += 360.0;
  }

  return current_position;
}