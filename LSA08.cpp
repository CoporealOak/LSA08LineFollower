#include "LSA08.h"

LSA08::LSA08(int ANPin, int JPin){
  analogPin = ANPin;
  junctionPin = JPin;
}

void LSA08::Initialise(){
  pinMode(analogPin, INPUT);
  pinMode(junctionPin, INPUT);
}

float LSA08::getPositionError(){
  float currentRead = 35.0;
  const float actualMin = 0.0;
  const float actualCentre = 560.0;
  const float actualMax = 1875.0;

  float RawAnalogRead = analogRead(analogPin);

  if(RawAnalogRead < actualCentre){
    currentRead = (RawAnalogRead - actualMin) * (35.0 / (actualCentre - actualMin));
  }
  else{
    currentRead = 35.0 + (RawAnalogRead - actualCentre) * (35.0 / (actualMax - actualCentre));
  }

  if(currentRead < 0.0)
    currentRead = 0.0;
  else if(currentRead > 70.0)
    currentRead = 70.0;

  if(currentRead >= 34.0 && currentRead <= 35.5)
    currentRead = 35.0;

  float currentError = currentRead - 35.0;
  return currentError / 10.0;
}

bool LSA08::isLineLost(){
  return analogRead(analogPin) > 2600;
}

bool LSA08::isJunction(){
  return digitalRead(junctionPin) == HIGH;
}