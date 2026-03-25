#ifndef STATECONTROL_H
#define STATECONTROL_H

#include <Arduino.h>  
#include "LSA08.h"
#include "PIDControl.h"
#include "driveControl.h"

enum RobotState{
  FollowingLine, LostLine, Turn90L, Intersection, AtEndPoint, CrossingGap
};

enum Direction{
  Left, Right, Center
};

class StateControl{
  private: 
    RobotState currentState;
    RobotState previousState;
    Direction LastSeenSide;

    LSA08& sensors;
    PIDControl& pid;
    driveControl& drive;

    float searchSpeed;
    float pivotSpeed;

    unsigned long AntiLoopTimer;
    unsigned long LoopEscapeTimer;
    unsigned long TurnTimer;
    unsigned long EndPointTimer;
    unsigned long LineLostTime;

    bool ForceEscape;

    int EndPointLED;

    void determineState();
    void logLastPosition(float currentError);
    void executePID(float error);
    void handleLostLine();
    void handle90DegreeTurn();
    void handleIntersection();
    void EndPointReached();
    void handleGap();

  public:
    StateControl(LSA08& sensorRef, PIDControl& pidRef, driveControl& driveRef, int LEDPinIn);
    void update();
    RobotState getState();
};

#endif