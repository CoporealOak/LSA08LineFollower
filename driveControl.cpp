#include <Arduino.h>
#include "driveControl.h"

driveControl::driveControl(int l_in1, int l_in2, int l_pwm, int r_in1, int r_in2, int r_pwm): leftMotor(l_in1, l_in2, l_pwm), rightMotor(r_in1, r_in2, r_pwm){
  defaultSpeedL = 140;
  defaultSpeedR = 140;
}

void driveControl::initialise(){
  leftMotor.Initialise();
  rightMotor.Initialise();
}

void driveControl::setDefaultSpeed(int speedL, int speedR){
  defaultSpeedL = speedL;
  defaultSpeedR = speedR;
}

void driveControl::steer(float PIDOutput){
  int leftSpeed = defaultSpeedL + PIDOutput;
  int rightSpeed = defaultSpeedR - PIDOutput;

  leftMotor.drive(leftSpeed);
  rightMotor.drive(rightSpeed);
}

void driveControl::stop(){
  leftMotor.drive(0);
  rightMotor.drive(0);
}

void driveControl::spin(int speed){
  leftMotor.drive(speed);
  rightMotor.drive(-speed);
}