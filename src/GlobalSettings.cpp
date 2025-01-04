#include "GlobalSettings.h"

const int NUM_MOTORS = 1;

// PID  Velocity constants
double kpVelo = 0.5;
double kiVelo = 0.0;
double kdVelo = 0.0;

// PID Position constants
double kpPos = 0.0;
double kiPos = 0.0;
double kdPos = 0;

int dir = 1;

String message = "";
String message_car = "";
std::vector<String> test_messages = {"", "", "", ""};

// interval for measurements
int interval_velocity = 50;
int interval_position = 1;
int interval_pid_velocity = 100;
int interval_velocity_info = 1000;

// Pin configurations for motors
// motorPin motorPin0 = motorPin(4, 5, 15, 2);    // C2, C1, L_PWM, R_PWM
// motorPin motorPin1 = motorPin(25, 33, 27, 26); // C2, C1, L_PWM, R_PWM
// motorPin motorPin2 = motorPin(21, 22, 18, 19); // C2, C1, L_PWM, R_PWM
// motorPin motorPin3 = motorPin(32, 35, 13, 14); // C2, C1, L_PWM, R_PWM

// Wheel[0] : FL
// Wheel[1] : FR
// Wheel[2] : RL
// Wheel[3] : RR
// pinA: C2
// pinB: C1
// Right > 0
// Left < 0


motorPin motorPin0 = motorPin(4, 5, 15, 2);  // pinA, pinB, L_PWM, R_PWM
// motorPin motorPin1 = motorPin(25, 33, 27, 26);  // pinA, pinB, L_PWM, R_PWM
motorPin motorPin1 = motorPin(13, 14, 27, 26);  // pinA, pinB, L_PWM, R_PWM
motorPin motorPin2 = motorPin(21, 22, 18, 19);  // pinA, pinB, L_PWM, R_PWM
// motorPin motorPin3 = motorPin(32, 35, 13, 14);  // pinA, pinB, L_PWM, R_PWM
motorPin motorPin3 = motorPin(32, 35, 25, 33);  // pinA, pinB, L_PWM, R_PWM

Wheel wheels[] = {
    Wheel(0, motorPin0.pinA, motorPin0.pinB, motorPin0.L_PWM, motorPin0.R_PWM),
    Wheel(1, motorPin1.pinA, motorPin1.pinB, motorPin1.L_PWM, motorPin1.R_PWM),
    Wheel(2, motorPin2.pinA, motorPin2.pinB, motorPin2.L_PWM, motorPin2.R_PWM),
    Wheel(3, motorPin3.pinA, motorPin3.pinB, motorPin3.L_PWM, motorPin3.R_PWM)};

Car mecanumCar(LX, LY, WHEEL_RADIUS, wheels);
// long time = 0;

// Declare the trigger interrupt function
void triggerW0() { wheels[0].triggerInterrupt(); };
void triggerW1() { wheels[1].triggerInterrupt(); };
void triggerW2() { wheels[2].triggerInterrupt(); };
void triggerW3() { wheels[3].triggerInterrupt(); };