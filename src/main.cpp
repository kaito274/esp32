#include "Motor.h"
#include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// #define VELOCITY 0
#define POSITION 1

// interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;


void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  // Attach the interrupt functions to the encoder pins
  attachInterrupt(digitalPinToInterrupt(wheels[0].getEncPinA()), triggerW1, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[1].getEncPinA()), triggerW2, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[2].getEncPinA()), triggerW3, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[3].getEncPinA()), triggerW4, RISING);

  // Enable the motor
  wheels[0].getMotor().Enable();

#ifdef VELOCITY  
  wheels[0].setTargetRPM(100);
#endif // VELOCITY

#ifdef POSITION
  wheels[0].setTargetPosition(333);
#endif // POSITION

}

void loop()
{ 
#ifdef VELOCITY
  // Velocity 
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    wheels[0].infoVelocity();
    wheels[0].tuningRPM();
    if (dir == 1) {
      wheels[0].getMotor().TurnRight(wheels[0].getPWM());
    } else {
      wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
    }
    previousMillis = currentMillis;
  }
#endif // VELOCITY
#ifdef POSITION
  // Position
  wheels[0].infoPosition();
  wheels[0].tuningPosition();
  if (wheels[0].getDirection() == LEFT_DIR) {
    wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
  } else {
    wheels[0].getMotor().TurnRight(wheels[0].getPWM());
  }
 #endif // POSITION
}

