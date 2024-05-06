#ifndef SEEDHARVESTER_H
#define SEEDHARVESTER_H

#include <Servo.h>

class SeedHarvester {
public:
  // gpin refer to *Servo at grabbing position
  // lft A and B refer to lifter motor pin
  SeedHarvester(int gpin, int lftA, int lftB, int Minlimit, int Maxlimit, int stepPin, int dirPin, int setzeropin);

  void setup(); // set pin and gripper position
  // stepper
  void linearDrive(double dis, int dir); // drive gripper position to disire position and direction
  void setZero(); // set the gripper position to the start
  // servo
  void grab(); // grab an object
  void release(); // release an object 
  // command function
  void singleHarvest_locking(); // harvest the object
  void singleRelease(); // release the harvested obj

  void stack();
  void preparing();
  void pull_up();
  void pull_down();
  void Stacking(bool next);

  void drop();
  void drop_down();
  void poping(bool next);

  void add_dis(int dis);
 
  // additional from old gripper
  void lifter_up(int pwm); // pull the gripper up
  void lifter_down(int pwm); // pull the gripper down

  void lifter_up(int pwm, int mms); // pull the gripper up with time
  void lifter_down(int pwm, int mms); // pull the gripper down with time

  void single_press(bool next);
private:
  int gap = 80;  //distance between seed in mm

  int gpin_;      // grabber pin (servo)
  int lftA_;     // lifter pin (ref A)
  int lftB_;     // lifter pin (ref B)
  int Minlimitpin_;  // ground limit pin
  int Maxlimitpin_;  // top limit pin
  int dirPin_;    // Direction pin for the step motor
  int stepPin_;   // Step pin for the step motor
  int setzeropin_; // limit pin to set zero for step motor

  int storage = 0;
  int manual_lock_dis = 440; // the initial pos in mm
  int max_dis = 440; // the max pos in mm

  int n_release = 0;

  bool Ldir = true; // go out from set zero pos
  bool Rdir = false; // go in to set zero pos

  int grbAng = 0;
  int relAng = 50;

  int pwm = 200;

  bool harvest = true;
  bool ready_to_stack = true;
  bool pulling = false;

  int stage = 0;
  int pop_stage = 0;
};

#endif  // SEEDHARVESTER_H