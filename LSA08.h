#ifndef LSA08_H
#define LSA08_H

#include <Arduino.h>

class LSA08{
  private:
    int analogPin, junctionPin;
  
  public:
    LSA08(int ANPin, int Jpin);
    void Initialise();
    bool isJunction();
    bool isLineLost();
    float getPositionError();
};

#endif