#include <Arduino.h>
#include "PIDControl.h"

PIDControl::PIDControl(float p, float i, float d, float Alpha){
  
  kp = p;
  ki = i;
  kd = d;
  alpha = Alpha;
  lastOut = 0.0;

  previousError = 0.0;
  integralSum = 0.0;
  previousFilteredDerivative = 0.0;

  lastTime = micros();
}

void PIDControl::setTunings(float p, float i, float d){
  kp = p;
  ki = i;
  kd = d;
  lastTime = micros();
}

float PIDControl::compute(float currentError){

  unsigned long int currentTime = micros();
  float timeDelta = (currentTime - lastTime)/1000000.0;
  float dynamicKp = kp;
  float dynamicKd = kd;
  
  if(timeDelta < 0.01){
    return lastOut;
  }

  if(abs(currentError) > 1.8){
    dynamicKp = kp * 2.0;
  }
  else if(abs(currentError) > 0.45 && abs(currentError) <= 1.8 && abs(currentError) > abs(previousError)){
    dynamicKd = kd * 2.5;
  }

  float P = dynamicKp * currentError;

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
  float D = filteredDerivative * dynamicKd;

  lastTime = currentTime;
  previousError = currentError;
  previousFilteredDerivative = filteredDerivative;
  lastOut = P + I + D;

  return P + I + D;
}

void PIDControl::reset(){
  integralSum = 0.0;
  previousError = 0.0;
  previousFilteredDerivative = 0.0;
  lastTime = micros();
  lastOut = 0.0;
}