#include "LSA08.h"
#include "driveControl.h"
#include "PIDControl.h"
#include "StateControl.h"
#include "Logger.h"

const int LSA_ANPin = 34;
const int LSA_JPin = 35;
const int endPointLED = 13;

PIDControl PID(1.0, 0.0, 0.0);
driveControl drive(25, 26, 33, 27, 14, 12);
LSA08 SensorArray(LSA_ANPin, LSA_JPin);
Logger debugLogger(50);

StateControl StateMachine(SensorArray, PID, drive, 13);

void setup(){
  Serial.begin(115200);

  pinMode(endPointLED, OUTPUT);
  digitalWrite(endPointLED, LOW);

  SensorArray.Initialise();
  drive.initialise();
  debugLogger.begin(115200);

  drive.setDefaultSpeed(150);

  delay(5000);

  PID.reset();
}

void loop(){
  StateMachine.update();

  float currentError = SensorArray.getPositionError();
  float pidCorrection = PID.compute(currentError);

  int baseSpeed = 150;
  int leftSpeed = baseSpeed + pidCorrection;
  int rightSpeed = baseSpeed - pidCorrection;

  debugLogger.logData(currentError, pidCorrection, leftSpeed, rightSpeed);
}