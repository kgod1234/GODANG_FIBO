#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
#include "pio_encoder.h"

class Motor {
public:
  Motor(int encoderPin, int INA_PIN, int INB_PIN, double kp, double ki, double kd);  // Constructor
  void setSpeed(int speed);                                                          // Set the speed of the motor
  void stop();                                                                       // Stop the motor
  void compute(float setPoint_RPM, float deltaTime);                                 // Compute the PID and update speed
  void TryFilter(float setPoint_RPM, float deltaTime);     
  void FindSpeedFromPWM(float pwm, float deltaTime);
  void setKP(float kp);                                                              // Set the Proportional gain
  void setKI(float ki);                                                              // Set the Integral gain
  void setKD(float kd);                                                              // Set the Derivative gain

  PioEncoder encoder;
  
private:
  float _vFilt;
  float _v, _u, _ffw;
  int _INA_PIN;
  int _INB_PIN;
  float _kp, _ki, _kd;
  float _vPrev;
  float _error, _lastError, _integral;
  int _prevCount = 0;
  const float _alpha = 0.1; //Closer to 0 more smmoth but slow
};

#endif