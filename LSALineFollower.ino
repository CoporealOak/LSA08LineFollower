#include "LSA08.h"
#include "driveControl.h"
#include "PIDControl.h"
#include "StateControl.h"
#include "Logger.h"

const int LSA_ANPin = 34;
const int LSA_JPin = 35;
const int endPointLED = 13;

PIDControl PID(24.0, 0.0, 1.5);
driveControl drive(27, 14, 12, 25, 26, 33);
LSA08 SensorArray(LSA_ANPin, LSA_JPin);

StateControl StateMachine(SensorArray, PID, drive, 13);

void setup(){
  Serial.begin(115200);

  pinMode(endPointLED, OUTPUT);
  digitalWrite(endPointLED, LOW);

  SensorArray.Initialise(); 
  drive.initialise();

  drive.setDefaultSpeed(126, 126);

  delay(2000);

  PID.reset();
}

void loop(){
  StateMachine.update();
  Serial.println(analogRead(LSA_ANPin));
}