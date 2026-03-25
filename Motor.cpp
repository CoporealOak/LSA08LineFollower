#include <Arduino.h>
#include "Motor.h"

Motor::Motor(const int in1, const int in2, const int pwm){
  in1Pin = in1;
  in2Pin = in2;
  pwmPin = pwm;
}

void Motor::Initialise(){                 //Initialise pins and stop the bot initially
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(pwmPin, OUTPUT);

  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(pwmPin , 0);
}

void Motor::drive(int speed){             // Use the speed to define direction and speed of the wheels
  speed = constrain(speed, -255, 255);

  if(speed > 0){
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  }
  else if(speed < 0){
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, abs(speed));
  }
  else{
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin , 0);
  }
}