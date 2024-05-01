#ifndef BALL_SHOOTER_H
#define BALL_SHOOTER_H

#include <Servo.h>

class BallShooter {
public:
    BallShooter(int servoPin, int limitSwitchPin, int INA, int INB, int grb_ref1, int grb_ref2, int stepPin, int dirPin);
    // debug
    void servo();
    void limitswitch();
    void stepper_cw();
    void stepper_ccw();
    void stepper_stop();
    // part in control function
    void motor(int pwm);
    void motor_stop();
    void wheel(int pwm);
    void wheel_stop();

    //additional function for control
    void setup();
    void preparing();
    void grab();
    void shoot();

private:
    Servo s;
    int servoPin_;
    int limitSwitchPin_;
    int INA_;
    int INB_;
    int grb_ref1;
    int grb_ref2;
    int stepPin_;
    int dirPin_;

    static const int stepsPerRevolution = 200;
};

#endif // BALL_SHOOTER_H