#include "BallShooter.h"
#include <Arduino.h>

BallShooter::BallShooter(int servoPin, int limitSwitchPin, int INA, int INB, int grb_ref1, int grb_ref2, int stepPin,
                         int dirPin)
  : servoPin_(servoPin)
  , limitSwitchPin_(limitSwitchPin)
  , INA_(INA)
  , INB_(INB)
  , grb_ref1(grb_ref1)
  , grb_ref2(grb_ref2)
  , stepPin_(stepPin)
  , dirPin_(dirPin)
{
}

void BallShooter::motor(int pwm)
{
  if (pwm > 0)
  {
    analogWrite(grb_ref1, 0);  // up
    analogWrite(grb_ref2, pwm);
  }
  else
  {
    analogWrite(grb_ref1, abs(pwm));
    analogWrite(grb_ref2, 0);  // down
  }
}

void BallShooter::motor_stop()
{
  analogWrite(grb_ref2, 0);  // stop
  analogWrite(grb_ref1, 0);
}

void BallShooter::setup()
{
  // pin setting and position of all component
  s.attach(servoPin_);
  pinMode(limitSwitchPin_, INPUT_PULLUP);
  pinMode(INA_, OUTPUT);
  pinMode(INB_, OUTPUT);
  pinMode(grb_ref1, OUTPUT);
  pinMode(grb_ref2, OUTPUT);
  pinMode(stepPin_, OUTPUT);
  pinMode(dirPin_, OUTPUT);
  Serial.begin(115200);
  s.write(180);
  delay(200);
  // keep motor from the ground
  motor_stop();
  delay(100);
  // adjust ball holder position
  digitalWrite(dirPin_, LOW);
  while (digitalRead(limitSwitchPin_) == 1)
  {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
  digitalWrite(dirPin_, HIGH);
  for (int i = 0; i < 880; ++i)
  {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
}
// servo debug
void BallShooter::servo()
{
  s.write(140);
  delay(2000);
  s.write(180);
  delay(2000);
  s.write(140);
  delay(2000);
}
// limit debug
void BallShooter::limitswitch()
{
  if (digitalRead(limitSwitchPin_) == 1)
  {
    Serial.println("1");
  }
  else
  {
    Serial.println("0");  // trig
  }
}

void BallShooter::stepper_cw()
{
  digitalWrite(dirPin_, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++)
  {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
}

void BallShooter::stepper_ccw()
{
  digitalWrite(dirPin_, LOW);
  for (int i = 0; i < 4.2 * stepsPerRevolution; i++)
  {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
}

void BallShooter::stepper_stop()
{
  digitalWrite(dirPin_, LOW);
  for (int i = 0; i < 2 * stepsPerRevolution; i++)
  {
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
}

void BallShooter::wheel(int pwm)
{  // ON fly wheel
  analogWrite(INA_, pwm);
  analogWrite(INB_, 0);
}

void BallShooter::wheel_stop()
{  // OFF fly wheel
  analogWrite(INA_, 0);
  analogWrite(INB_, 0);
}

// additional function for control
void BallShooter::preparing()
{  // keep the grabber on the ground to grab a ball

  motor(-230);  // down
  delay(1600);

  motor_stop();  // stop motor
  delay(200);

  s.write(140);  // keep the grabber avaiable
}

void BallShooter::grab()
{  // grab the ball and keep high from the ground
  // grab by servo
  s.write(180);
  delay(500);
  //  s.write(120);
  // adjust motor position
  motor(230);  // up
  delay(1700);
  // release the ball on holder
  // s.write(160);
  motor_stop();
  delay(100);
}

void BallShooter::shoot()
{
  s.write(140);
  //  delay(500);
  // On fly wheel at full speed
  int power = 0;
  wheel(power);
  // deliver a ball to fly wheel
  unsigned long long int start = millis();
  digitalWrite(dirPin_, LOW);
  while (digitalRead(limitSwitchPin_) == 1)
  {
    if (power > 255)
    {
      power = 255;
    }
    if (millis() - start == 10)
    {
      start = millis();
      power += 70;
      wheel(power);
    }
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
  // slow down the wheel
  //   for(int i = 255 ; i > 0; i-52){
  //     wheel(i); delay(100);
  //   }
  // stop
  wheel_stop();
  s.write(180);
  // set holder back to same old pos
  digitalWrite(dirPin_, HIGH);

  for (int i = 0; i < 880; ++i)
  {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(1000);
  }
}