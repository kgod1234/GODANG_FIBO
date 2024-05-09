#include "SeedHarvester.h"
#include <Arduino.h>
Servo GrabServo;

SeedHarvester::SeedHarvester(int gpin, int lftA, int lftB, int Minlimit, int Maxlimit, int dirPin, int stepPin, int setzeropin)
  : gpin_(gpin), lftA_(lftA), lftB_(lftB), Minlimitpin_(Minlimit), Maxlimitpin_(Maxlimit) ,dirPin_(dirPin), stepPin_(stepPin), setzeropin_(setzeropin) {
  
}

void SeedHarvester::linearDrive(double dis, int dir) {
  int steps = dis / 0.285;
    if(!dir){
      steps = steps * -1;
    }
    stepper.runSpeed();
    stepper.moveTo(steps);
    stepper.runToPosition();
    stepper.setCurrentPosition(0);
    delay(500); // Optional delay after movement
}

void SeedHarvester::linearDriveSteady(double dis, int dir) {
  int stpd = 500;
  int rpt = dis / 0.285;
  digitalWrite(dirPin_, dir);
  for (int i = 0; i < rpt; i++) {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(stpd);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(stpd);
  }
  delay(500);
}

void SeedHarvester::setZero() {
  int pd = 500;
  digitalWrite(dirPin_, Rdir);
  unsigned long start = millis();
  while (digitalRead(setzeropin_) == 1) {
    if(millis() - start > 50){
      pd = 350;
    }
    else if(millis() - start > 100){
      pd = 300;
    }
    else if(millis() - start > 150){
      pd = 250;
    }
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(pd);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(pd);
  }
  delay(500);
}

void SeedHarvester::lifter_up(int pwm){

  while(digitalRead(Minlimitpin_)){ // up
    analogWrite(lftA_, 0);
    analogWrite(lftB_, pwm);
  }
  analogWrite(lftA_, 0);
  analogWrite(lftB_, 0);
  delay(100);
}

void SeedHarvester::lifter_down(int pwm){

  while(digitalRead(Maxlimitpin_)){ // down
    analogWrite(lftA_, pwm);
    analogWrite(lftB_, 0);
  }
  analogWrite(lftA_, 0);
  analogWrite(lftB_, 0);
  delay(100);
}

void SeedHarvester::lifter_up(int pwm, int mms){

  analogWrite(lftA_, 0); // up
  analogWrite(lftB_, pwm);
  delay(mms);
  analogWrite(lftA_, 0);
  analogWrite(lftB_, 0);
  delay(100);
}

void SeedHarvester::lifter_down(int pwm, int mms){

  analogWrite(lftA_, pwm); // down
  analogWrite(lftB_, 0);
  delay(mms);
  analogWrite(lftA_, 0);
  analogWrite(lftB_, 0);
  delay(100);
}

void SeedHarvester::grab() {
  GrabServo.write(grbAng);
  delay(500);
}

void SeedHarvester::release() {
  // delay(800);
  GrabServo.write(relAng);
  delay(500);
}

void SeedHarvester::setup(){
  // dc motor
  pinMode(lftA_, OUTPUT);
  pinMode(lftB_, OUTPUT);
  // limit switch
  pinMode(Minlimitpin_, INPUT_PULLUP);
  pinMode(Maxlimitpin_, INPUT_PULLUP);
  pinMode(setzeropin_, INPUT_PULLUP);
  // step motor at constructor
  // servo
  GrabServo.attach(gpin_);
  stepper.setSpeed(init_spd);
  stepper.setMaxSpeed(max_spd); // Set your desired maximum speed in steps per second
  stepper.setAcceleration(acc); // Set your desired acceleration in steps per second per second
  
  release();
  lifter_up(this->pwm);

  setZero();
  linearDrive(manual_lock_dis, Ldir);


  lifter_down(this->pwm);

  analogWrite(lftA_, 0);
  analogWrite(lftB_, 0);
  delay(100);
  release();
}

void SeedHarvester::stack(){ // stack in storage
  if(ready_to_stack && pulling){ // check if ready to stack and the seed is already grab
    if (storage == 6) { // prevent to store more than 6
      harvest = false; // telling haervester that no need to harvest
      return;
    }
    if (storage == 0){
      linearDrive(manual_lock_dis - 15, Rdir);
      manual_lock_dis = manual_lock_dis - 15;
      storage = storage + 1;
      ready_to_stack = false;
      stage = 2; 
      return;
    }
    else if(storage > 0 && storage < 5){
      linearDrive(manual_lock_dis - gap, Rdir);
    }
    else if(storage == 5){
      linearDrive(manual_lock_dis - gap, Rdir);
      pop_stage = 0;
      // stage = 0;
      harvest = false;
    }
    manual_lock_dis = manual_lock_dis - gap;
    storage = storage + 1;
  }
  ready_to_stack = false;
  stage = 2;
}

void SeedHarvester::preparing(){ // preparing to grab
    if( !ready_to_stack ){
      if(storage == 5){
        // lifter_down(this->pwm,500);
        release();
        // lifter_up(this->pwm);
        linearDrive(manual_lock_dis, Ldir);
        lifter_down(this->pwm);
        stage = 0;
        ready_to_stack = true;
        pulling = false;
        return;
      }
      release();
      linearDrive(manual_lock_dis, Ldir);
      lifter_down(this->pwm);
    }
    stage = 0;
    ready_to_stack = true;
    pulling = false;
}

void SeedHarvester::pull_up(){ // pull the seed up
  if(ready_to_stack && !pulling){
    if (storage == 6) {
      harvest = false;
      return;
    }
    else{
      grab();
      lifter_up(this->pwm);
      stage = 1;
      pulling = true;
      return;
    }
  }
}

void SeedHarvester::pull_down(){ // undo pull up
  if(ready_to_stack && pulling){
    lifter_down(this->pwm);
    release();
    pulling = false;
  }
}

void SeedHarvester::Stacking(bool next){ // to stack seed in harvest stage
  if(harvest){ // check if in harvest stage
    if(next){ // if forward stage
      stage++;
      if(stage == 1){
        pull_up(); // pulling the seed up
      }
      else if(stage == 2) {
        stack(); // stack in storage
      }
      else{
        preparing(); // prepare to grab
      }
    }
    else{ // if redo stage
      if(stage > 0){
        stage--;
      }
      if(stage == 0){
        GrabServo.write(100);
        delay(300);
        lifter_down(this->pwm);
        pulling = false;
      }
      else{
        storage--;
        preparing();
        manual_lock_dis = manual_lock_dis + gap;
      }
    }
  }
}

void SeedHarvester::drop() { // to the droping position
  if(!harvest){ // check if not in harvest stage
    if (storage == 0) {
      harvest = true;
      return;
    }
      linearDrive(manual_lock_dis, Ldir);
  }
}

void SeedHarvester::drop_down() { // drop the seed down in deploying stage
  if(!harvest){ // check if not in harvest stage
    lifter_down(this->pwm, 200); // drop down fr
    GrabServo.write(100);
    delay(1500);
    lifter_up(this->pwm); // take the gripper up
    manual_lock_dis = manual_lock_dis + gap;
    storage = storage - 1;
    if (storage == 1) { // moving to next seed to deploy (last one)
      setZero();
      manual_lock_dis = max_dis;
      // harvest = true;
      stage = 0;
      return;
    } 
    if (storage == 0) { // moving to next seed to deploy (last one)
      setZero();
      stage = 0;
      manual_lock_dis = max_dis;
      harvest = true;
      return;
    } 
    linearDrive(manual_lock_dis, Rdir);// moving to next seed to deploy
  }
}

void SeedHarvester::poping(bool next){// release the seed
  if(!harvest){
        if(next){
          pop_stage++;
          if(pop_stage == 1){
              drop();
          }
          else if(pop_stage == 2){
              drop_down();
          }
          else if(pop_stage == 3){
              grab();
              pop_stage = 0;
          }
       }
       else{
        pop_stage--;
        if(pop_stage == -1){
          release();
          pop_stage = 2;
        }
        else if(pop_stage == 0){
          release();
          linearDrive(manual_lock_dis, Rdir);
          pop_stage = 2;
        }
        else{
          return;
        }
      }
    }
}

void SeedHarvester::add_dis(int dis){ // adding the traveling position
    manual_lock_dis = manual_lock_dis + dis;
}

void SeedHarvester::single_press(bool next){ // for pressing
  if(harvest == true){// check if in harvest stage
    Stacking(next); // next tell u to move to next stage if it true
  }
  else{
    poping(next);
  }
}