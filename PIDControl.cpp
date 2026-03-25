#include <Arduino.h>
#include "PIDControl.h"

PIDControl::PIDControl(float p, float i, float d, float Alpha){
  
  kp = p;
  ki = i;
  kd = d;
  alpha = Alpha;

  previousError = 0.0;
  integralSum = 0.0;
  previousFilteredDerivative = 0.0;

  lastTime = micros();
}

void PIDControl::setTunings(float p, float i, float d){
  kp = p;
  ki = i;
  kd = d;
  lastTime = 0;
}

float PIDControl::compute(float currentError){

  unsigned long int currentTime = micros();
  float timeDelta = (currentTime - lastTime)/1000000.0;
  if(timeDelta <= 0)
    timeDelta = 0.0001;

  float P = kp * currentError;

  integralSum = integralSum + (currentError * timeDelta);
  float integralLimit = 255.0;

  if(integralSum > integralLimit){
    integralSum = integralLimit;
  }else if(integralSum < -integralLimit){
    integralSum = -integralLimit;
  }

  float I = integralSum * ki;

  float rawDerivative = (currentError - previousError) / timeDelta;
  float filteredDerivative = (alpha * rawDerivative) + (1.0 - alpha) * previousFilteredDerivative;
  float D = filteredDerivative * kd;

  lastTime = currentTime;
  previousError = currentError;
  previousFilteredDerivative = filteredDerivative;

  return P + I + D;
}

void PIDControl::reset(){
  integralSum = 0.0;
  previousError = 0.0;
  previousFilteredDerivative = 0.0;
  lastTime = micros();
}