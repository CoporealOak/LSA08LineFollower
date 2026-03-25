#include <Arduino.h>
#include "Logger.h"

Logger::Logger(int intervalInms){
  printInterval = intervalInms;
  lastPrintTime = millis();
}

void Logger::begin(long baudRate){
  Serial.begin(baudRate);
}

void Logger::logData(float error, float PIDOutput, int leftSpeed, int rightSpeed){
  unsigned long currentTime = millis();

  if(currentTime - lastPrintTime >= printInterval){
    lastPrintTime = currentTime;

    Serial.print(error);
    Serial.print(", ");

    Serial.print(PIDOutput);
    Serial.print(", ");

    Serial.print(leftSpeed);
    Serial.print(", ");

    Serial.println(rightSpeed);
  }
}