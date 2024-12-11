#include "GlobalSettings.h"

const int NUM_MOTORS = 1;

// PID constants
double kp = 1.0;
double kd = 0;
double ki = 0.0;

// Pin configurations for motors
motorPin motorPin1 = motorPin(13, 26, 27, 14, 12); // pinA, pinB, EN, L_PWM, R_PWM 

// Declare the trigger interrupt function
Wheel wheelsSpeed[] = {
  Wheel(motorPin1.pinA, motorPin1.pinB, motorPin1.EN, motorPin1.L_PWM, motorPin1.R_PWM, 0, 0, kp, kd, ki)
};

void triggerW1A() {wheelsSpeed[0].triggerA();};
void triggerW1B() {wheelsSpeed[0].triggerB();};