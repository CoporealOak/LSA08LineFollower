#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H
#include <Arduino.h>
#include "Motor.h"

class driveControl{
  private:
    Motor leftMotor;
    Motor rightMotor;
    int defaultSpeed;

  public:
    driveControl(int l_in1, int l_in2, int l_pwm, int r_in1, int r_in2, int r_pwm);
    void initialise();
    void setDefaultSpeed(int speed);
    void steer(float PIDOutput);
    void stop();
};

#endif