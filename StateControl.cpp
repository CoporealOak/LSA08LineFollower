#include "StateControl.h"

StateControl::StateControl(LSA08& sensorRef, PIDControl& pidRef, driveControl& driveRef, int LEDPinIn)
  : sensors(sensorRef), pid(pidRef), drive(driveRef) {
  currentState = FollowingLine;
  LastSeenSide = Center;

  searchSpeed = 100;
  pivotSpeed = 130;
  AntiLoopTimer = 0;
  LoopEscapeTimer = millis();
  TurnTimer = 0;
  ForceEscape = false;
  EndPointLED = LEDPinIn;
  pinMode(EndPointLED, OUTPUT);
}

void StateControl::determineState() {
  if (currentState == Turn90L) {
    if (millis() - TurnTimer < 150) {
      return;
    } else if (sensors.isLineLost()) {
      if (millis() - TurnTimer > 1500) {
        currentState = LostLine;
      }
      return;
    } else {
      currentState = FollowingLine;
      AntiLoopTimer = millis();
      return;
    }
  }

  if (millis() - LoopEscapeTimer > 17000) {
    ForceEscape = true;
  }

  if (sensors.isJunction()) {
    if (millis() - AntiLoopTimer > 600) {

      if (ForceEscape) {
        TurnTimer = millis();
        currentState = Turn90L;
        LastSeenSide = Left;
        ForceEscape = false;
        LoopEscapeTimer = millis();
      } else {
        if (currentState != Intersection && currentState != AtEndPoint) {
          EndPointTimer = millis();
          currentState = Intersection;
          LastSeenSide = Center;
          LoopEscapeTimer = millis();
        } else if (currentState == Intersection && (millis() - EndPointTimer > 800)) {
          currentState = AtEndPoint;
        }
      }

    } else {
      currentState = FollowingLine;
    }
  }

  else if (sensors.isLineLost()) {

    if(currentState != CrossingGap && currentState != LostLine && currentState != Turn90L){
      currentState = CrossingGap;
      LineLostTime = millis();
    }
    
    else if(currentState == CrossingGap){
      if (LastSeenSide == Center) {
        if (millis() - LineLostTime > 800) {
          currentState = LostLine;
        }
      }

      else {
        if (millis() - LineLostTime > 150) {
          
          if(millis() - AntiLoopTimer < 1000){
            currentState = Turn90L;
            TurnTimer = millis();
            AntiLoopTimer = 0;
            LastSeenSide = Left;
          } else {
            currentState = LostLine;
          }
        }
      }
    }
    
    else if(currentState == Turn90L){
      currentState = LostLine;
    }
  }

  else {
    currentState = FollowingLine;
  }

  if (previousState == Intersection && currentState != Intersection && currentState != AtEndPoint) {
    AntiLoopTimer = millis();
  }

  previousState = currentState;
}

void StateControl::logLastPosition(float currentError) {
  if (currentError < -0.5)
    LastSeenSide = Left;
  else if (currentError > 0.5)
    LastSeenSide = Right;
  else
    LastSeenSide = Center;
}

void StateControl::handleLostLine() {
  if (LastSeenSide == Left)
    drive.spin(-searchSpeed);
  else
    drive.spin(searchSpeed);
}

void StateControl::handleGap() {
  drive.steer(0.0);
}

void StateControl::handle90DegreeTurn() {
  drive.spin(-pivotSpeed);
}

void StateControl::handleIntersection() {
  drive.steer(0.0);
}

void StateControl::executePID(float error) {
  float SteeringCorrection = pid.compute(error);
  drive.steer(SteeringCorrection);
}

void StateControl::EndPointReached() {
  drive.stop();
  digitalWrite(EndPointLED, HIGH);
}

void StateControl::update() {
  float currentErr = sensors.getPositionError();
  if (!sensors.isLineLost()) {
    logLastPosition(currentErr);
  }

  determineState();

  switch (currentState) {
    case FollowingLine:
      executePID(currentErr);
      break;
    case LostLine:
      handleLostLine();
      break;
    case Turn90L:
      handle90DegreeTurn();
      break;
    case Intersection:
      handleIntersection();
      break;
    case AtEndPoint:
      EndPointReached();
      break;
    case CrossingGap:
      handleGap();
      break;
    default:
      drive.stop();
      break;
  }
}