#include "StateControl.h"

StateControl::StateControl(LSA08& sensorRef, PIDControl& pidRef, driveControl& driveRef, int LEDPinIn)
  : sensors(sensorRef), pid(pidRef), drive(driveRef) {
  currentState = FollowingLine;
  LastSeenSide = Center;

  searchSpeed = 125;
  pivotSpeed = 125;
  AntiLoopTimer = 0;
  LoopEscapeTimer = millis();
  TurnTimer = 0;
  ForceEscape = false;
  EndPointLED = LEDPinIn;
  pinMode(EndPointLED, OUTPUT);
}

void StateControl::determineState() {
  float currentErr = sensors.getPositionError();

  // 1. TURN GUARD: If we are already turning, stay in this state until finished.
  if (currentState == Turn90L) {
    if (millis() - TurnTimer < 300) {
      return;
    } else if (sensors.isLineLost()) {
      if (millis() - TurnTimer > 600) {
        currentState = LostLine;
      }
      return;
    } else {
      currentState = FollowingLine;
      AntiLoopTimer = millis();
      return;
    }
  }

  // 2. JUNCTION DETECTION: Distinguishes between Cross-intersections (X) and Corners (L).
  if (sensors.isJunction()) {
    if (millis() - AntiLoopTimer > 150) {

      // If the error is high, it's a 90-degree corner (L-junction).
      if (abs(currentErr) > 2.3) { 
        currentState = Turn90L;
        TurnTimer = millis();
        // Crucial: We do NOT set LastSeenSide to Center here. 
        // We keep the side we were leaning towards so handle90DegreeTurn knows where to spin.
      } 
      // If error is low, it's an X-intersection or the End Point.
      else {
        if (currentState != Intersection && currentState != AtEndPoint) {
          EndPointTimer = millis();
          currentState = Intersection;
          LastSeenSide = Center; // Force straight movement through the X
          LoopEscapeTimer = millis();
        } else if (currentState == Intersection && (millis() - EndPointTimer > 190)) {
          currentState = AtEndPoint;
        }
      }
    }
  }

  // 3. GAP DETECTION: Handles straight gaps and missed turns.
  else if (sensors.isLineLost()) {
    if (currentState != CrossingGap && currentState != LostLine && currentState != Turn90L) {
      currentState = CrossingGap;
      LineLostTime = millis();
    } 
    else if (currentState == CrossingGap) {
      // Increased timeout (300ms/450ms) to stop the "jerking" during straight gaps.
      int gapTimeout = (LastSeenSide == Center) ? 180 : 120;

      if (millis() - LineLostTime > gapTimeout) {
        // If we lost the line very soon after an intersection, it was a 90-degree turn.
        if (millis() - AntiLoopTimer < 400 && LastSeenSide != Center) {
          currentState = Turn90L;
          TurnTimer = millis();
          AntiLoopTimer = 0;
        } else {
          currentState = LostLine;
        }
      }
    }
  }

  // 4. THE LATCH: This is why your junction pin "stopped working."
  // We only revert to FollowingLine if we aren't currently processing an Intersection or EndPoint.
  else {
    if (currentState != Intersection && currentState != AtEndPoint) {
      currentState = FollowingLine;
    }
  }

  // 5. CLEANUP: Handles transition out of intersections.
  if (previousState == Intersection && currentState != Intersection && currentState != AtEndPoint) {
    AntiLoopTimer = millis();
  }

  previousState = currentState;
}

void StateControl::logLastPosition(float currentError) {
  if (currentError < -0.7)
    LastSeenSide = Left;
  else if (currentError > 0.7)
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
  if (LastSeenSide == Left) {
    drive.spin(-pivotSpeed);
  } else if (LastSeenSide == Right) {
    drive.spin(pivotSpeed);
  } else {
    drive.steer(0.0);
  }
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

  if(currentState != AtEndPoint){
    digitalWrite(EndPointLED, LOW);
  }

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