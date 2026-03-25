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
  float RawAnalogRead = analogRead(analogPin);
  if(RawAnalogRead > 3900){
    return 0.0;
  }
  float currentRead = (RawAnalogRead / 3685.0) * 70;
  float currentError = currentRead - 35;
  return currentError / 10.0;
}

bool LSA08::isLineLost(){
  return analogRead(analogPin) > 3900;
}

bool LSA08::isJunction(){
  return digitalRead(junctionPin) == HIGH;
}