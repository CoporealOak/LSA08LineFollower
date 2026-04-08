#ifndef PIDCONTROL_H
#define PIDCONTROL_H
#include <Arduino.h>

class PIDControl{
  private:
    float kp, ki, kd, previousError, integralSum, previousFilteredDerivative, alpha, lastOut;
    unsigned long int lastTime;
  
  public:
    PIDControl(float p, float i, float d, float Alpha = 0.55);
    void setTunings(float p, float i, float d);
    float compute(float currentError);
    void reset();
};

#endif