#include "GlobalSettings.h"

const int NUM_MOTORS = 1;

// PID constants
double kp = 1.0;
double ki = 0.0;
double kd = 0;

int dir = 1;

// Pin configurations for motors
motorPin motorPin1 = motorPin(4, 5, 15, 2); // pinA, pinB, L_PWM, R_PWM 
motorPin motorPin2 = motorPin(21, 22, 18, 19); // pinA, pinB, L_PWM, R_PWM
motorPin motorPin3 = motorPin(27, 26, 13, 14); // pinA, pinB, L_PWM, R_PWM
motorPin motorPin4 = motorPin(32, 35, 25, 33); // pinA, pinB, L_PWM, R_PWM

// Declare the trigger interrupt function
Wheel wheelsSpeed[] = {
  Wheel(motorPin1.pinA, motorPin1.pinB, motorPin1.L_PWM, motorPin1.R_PWM, 0, 0, kp, ki, kd),
  Wheel(motorPin2.pinA, motorPin2.pinB, motorPin2.L_PWM, motorPin2.R_PWM, 0, 0, kp, ki, kd),
  Wheel(motorPin3.pinA, motorPin3.pinB, motorPin3.L_PWM, motorPin3.R_PWM, 0, 0, kp, ki, kd),
  Wheel(motorPin4.pinA, motorPin4.pinB, motorPin4.L_PWM, motorPin4.R_PWM, 0, 0, kp, ki, kd)
};

void triggerW1A() {wheelsSpeed[0].triggerA();};
void triggerW1B() {wheelsSpeed[0].triggerB();};
void triggerW2A() {wheelsSpeed[1].triggerA();};
void triggerW2B() {wheelsSpeed[1].triggerB();};
void triggerW3A() {wheelsSpeed[2].triggerA();};
void triggerW3B() {wheelsSpeed[2].triggerB();};
void triggerW4A() {wheelsSpeed[3].triggerA();};
void triggerW4B() {wheelsSpeed[3].triggerB();};