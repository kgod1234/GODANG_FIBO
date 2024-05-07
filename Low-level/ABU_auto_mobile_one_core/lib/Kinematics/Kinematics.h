#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "math.h"
class Kinematics {
private:

public:
  struct Robot {
    float wheel_diameter;
    float lx;
    float ly;
    float wheel_circumference;
    float angular_to_rpm;
  };

  Robot robot;

  struct RPM {
    float RPM_FL;
    float RPM_FR;
    float RPM_BL;
    float RPM_BR;
  };

  struct Velocity {
    float vx;
    float vy;
    float wz;
  };

  struct Position {
    double x;
    double y;
    double theta;  //in degree
  };

  Position current_position;

  Kinematics(float wheel_diameter, float lx, float ly);
  RPM Inverse_Kinematics(float vx, float vy, float wz);
  Velocity Forward_Kinematics_Velocity(float RPM_FL, float RPM_FR, float RPM_BL, float RPM_BR);
  Position Forward_Kinematics_Position(float RPM_FL, float RPM_FR, float RPM_BL, float RPM_BR, Position current_position, float deltaTime);
};

#endif