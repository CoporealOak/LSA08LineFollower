#include "LSA08.h"
#include "driveControl.h"
#include "PIDControl.h"
#include "StateControl.h"
#include "Logger.h"

const int LSA_ANPin = 34;
const int LSA_JPin = 35;
const int endPointLED = 13;

PIDControl PID(30.0, 0.0, 0.8);
driveControl drive(27, 14, 12, 25, 26, 33);
LSA08 SensorArray(LSA_ANPin, LSA_JPin);

StateControl StateMachine(SensorArray, PID, drive, 13);

void setup(){
  Serial.begin(115200);

  pinMode(endPointLED, OUTPUT);
  digitalWrite(endPointLED, LOW);

  SensorArray.Initialise(); 
  drive.initialise();

  drive.setDefaultSpeed(130, 130);

  delay(5000);

  PID.reset();
}

void loop(){
  StateMachine.update();
}