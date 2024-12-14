#include "Motor.h"
#include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// volatile long posi = 0; // Position updated by the encoder
// volatile long encoderValues = 0; // Encoder value updated by the encoder

// // PID constants
// double kp_ = 1.0;
// double kd_ = 0.0;
// double ki_ = 0.0;
// // Declare PID variables
// double input = 0;      // Current position (input for PID)
// double output = 0;     // Output from PID (motor speed)
// double setpoint = 0;   // Target position for the motor
// // Define a type for function pointers (callback functions)
// typedef void (*CallbackFunction)();
// void updateEncoderB();
// void updateEncoderA();

// #define ENC_IN_A 4
// #define ENC_IN_B 5

// // Create a PID instance
// PID myPID(&input, &output, &setpoint, kp_, ki_, kd_, DIRECT);
// Motor motor1(15, 2); // EN, L_PWM, R_PWM

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  // Attach the interrupt functions to the encoder pins
  attachInterrupt(digitalPinToInterrupt(wheels[0].getEncPinA()), triggerW1, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[1].getEncPinA()), triggerW2, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[2].getEncPinA()), triggerW3, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[3].getEncPinA()), triggerW4, RISING);


  wheels[0].getMotor().Enable();
  

  // TODO: TEST
  wheels[0].setTargetRPM(100);
  wheels[0].setTargetPosition(333);
}

void loop()
{ 

  // Velocity 
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    wheels[0].tuningRPM();
    wheels[0].infoVelocity();
    if (dir == 1) {
      wheels[0].getMotor().TurnRight(wheels[0].getPWM());
    } else {
      wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
    }
    previousMillis = currentMillis;
  }

  // // Position
  // wheels[0].tuningPosition();
  // wheels[0].infoPosition();
  // if (wheels[0].getDirection() == LEFT_DIR) {
  //   wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
  // } else {
  //   wheels[0].getMotor().TurnRight(wheels[0].getPWM());
  // }
}

