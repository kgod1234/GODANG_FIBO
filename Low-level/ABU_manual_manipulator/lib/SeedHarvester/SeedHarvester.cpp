#include "SeedHarvester.h"
#include <Arduino.h>
Servo GrabServo;

SeedHarvester::SeedHarvester(int gpin, int lftA, int lftB, int Minlimit, int Maxlimit, int dirPin, int stepPin, int setzeropin)
  : gpin_(gpin), lftA_(lftA), lftB_(lftB), Minlimitpin_(Minlimit), Maxlimitpin_(Maxlimit) ,dirPin_(dirPin), stepPin_(stepPin), setzeropin_(setzeropin) {
  
}

void SeedHarvester::linearDrive(double dis, int dir) {
  double rpt = dis / 0.285;
  for (int i = 0; i < rpt; i++) {
    digitalWrite(dirPin_, dir);
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(700);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(700);
  }
  delay(500);
}

void SeedHarvester::setZero() {
  while (digitalRead(setzeropin_) == 1) {
    digitalWrite(dirPin_, Rdir);
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(700);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(700);
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

void SeedHarvester::singleRelease() {
  if (storage == 0) {
    harvest = true;
    return;
  }
  if (storage == 1) {
//    GrabServo.write(85);
//    delay(500);
//    setZero();
//    grab();
    delay(500);
    linearDrive(manual_lock_dis, Ldir);
    lifter_down(this->pwm, 800);
    release();
    delay(200);
    // setZero();
    storage = storage - 1;
    harvest = true;
    return;
  } 
  else if (storage <= 6 && storage > 1) {
    linearDrive(manual_lock_dis, Ldir);
    lifter_down(this->pwm, 800);
    release();
    delay(2000);
  
    GrabServo.write(85);
    delay(500);

    lifter_up(this->pwm);

    
    if(storage > 2){
      linearDrive(gap + manual_lock_dis, Rdir);
    }
    else if(storage == 2){
      setZero();
    }
    grab();
  } 
  manual_lock_dis = manual_lock_dis + gap;
  if(manual_lock_dis > 455){
    manual_lock_dis = 455;
  }
  storage = storage - 1;
}

void SeedHarvester::singleHarvest_locking() {
  if (storage == 6) {
    return;
    harvest = false;
  }

  if (storage == 5) {
    // set pos of gripper
    release();
    linearDrive(manual_lock_dis, Ldir);
    lifter_down(this->pwm);
    delay(2000);
    // grabbing stage
    grab();
    lifter_up(this->pwm,1000);
    linearDrive(manual_lock_dis - gap, Rdir);
    harvest = false;
  }

  else if (storage > 0 && storage < 5) {
    // set pos of gripper
    // release();

    release();

    // lifter_up(this->pwm);
    linearDrive(manual_lock_dis, Ldir);
    
    // GrabServo.write(relAng);
    lifter_down(this->pwm);
    delay(2000);
    // grabbing stage
    grab();
    lifter_up(this->pwm);
    linearDrive(manual_lock_dis - gap, Rdir);
  } 
  
  else if (storage == 0){
    // set pos of gripper
    GrabServo.write(relAng);
    // grabbing stage
    delay(2000);
    grab();
    lifter_up(this->pwm);
    setZero();
    storage = storage + 1;
    return;
  }

  // some pos cal
    manual_lock_dis = manual_lock_dis - gap;
    storage = storage + 1;
}

void SeedHarvester::setup(){
  // dc motor
  pinMode(lftA_, OUTPUT);
  pinMode(lftB_, OUTPUT);
  // limit switch
  pinMode(Minlimitpin_, INPUT_PULLUP);
  pinMode(Maxlimitpin_, INPUT_PULLUP);
  pinMode(setzeropin_, INPUT_PULLUP);
  // step motor
  pinMode(dirPin_, OUTPUT);
  pinMode(stepPin_, OUTPUT);
  // servo
  GrabServo.attach(gpin_);
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

void SeedHarvester::stack(){
  if(ready_to_stack && pulling){
    if (storage == 6) {
      harvest = false;
      return;
    }
    if (storage == 0){
      setZero();
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
      harvest = false;
    }
    manual_lock_dis = manual_lock_dis - gap;
    storage = storage + 1;
  }
  ready_to_stack = false;
  stage = 2;
}

void SeedHarvester::preparing(){
    if( !ready_to_stack ){
      release();
      linearDrive(manual_lock_dis, Ldir);
      lifter_down(this->pwm);
    }
    stage = 0;
    ready_to_stack = true;
    pulling = false;
}

void SeedHarvester::pull_up(){
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

void SeedHarvester::pull_down(){
  if(ready_to_stack && pulling){
    lifter_down(this->pwm);
    release();
    pulling = false;
  }
}

void SeedHarvester::Stacking(bool next){
  if(harvest){
    if(next){
      stage++;
      if(stage == 1){
        pull_up();
      }
      else if(stage == 2) {
        stack();
      }
      else{
        preparing();
      }
    }
    else{
      if(stage > 0){
        stage--;
      }
      if(stage == 0){
        pull_down();
      }
      else{
        storage--;
        preparing();
        manual_lock_dis = manual_lock_dis + gap;
      }
    }
  }
}

void SeedHarvester::drop() {
  if(!harvest){
    if (storage == 0) {
      harvest = true;
      return;
    }
      linearDrive(manual_lock_dis, Ldir);
  }
}

void SeedHarvester::drop_down() {
  if(!harvest){
    lifter_down(this->pwm, 800);
    release();
    lifter_up(this->pwm);
    manual_lock_dis = manual_lock_dis + gap;
    storage = storage - 1;
    if (storage == 1) {
      setZero();
      manual_lock_dis = 435;
      return;
    } 
    linearDrive(manual_lock_dis, Rdir);
    if (storage == 0) {
      harvest = true;
      return;
    }
  }
}

void SeedHarvester::locking() {
  if(!harvest){
    grab();
  }
}

void SeedHarvester::poping(){
  if(!harvest){
      pop_stage++;
    if(pop_stage == 1){
        drop();
    }
    else if(pop_stage == 2){
        drop_down();
    }
    else if(pop_stage == 3){
        locking();
        pop_stage = 0;
    }
  }
}

void SeedHarvester::add_dis(bool log, int dis){
  if(log){
      manual_lock_dis = manual_lock_dis + dis;
  }
  else{
    manual_lock_dis = manual_lock_dis - dis;
  }
}

void SeedHarvester::single_press(bool next){
  if(harvest == true){
    Stacking(next);
  }
  else{
    poping();
  }
}