#ifndef LOGGER_H
#define LOGGER_H
#include <Arduino.h>

class Logger{
  private:
    unsigned long lastPrintTime;
    int printInterval;

  public:
    Logger(int intervalInms);
    void begin(long baudRate);
    void logData(float error, float PIDControl, int leftSpeed, int rightSpeed);
};

#endif