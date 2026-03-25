#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

class Motor{
  private:
    int in1Pin, in2Pin, pwmPin;

  public:
    Motor(int in1, int in2, int pwm);

    void Initialise();

    void drive(int speed);
};

#endif 