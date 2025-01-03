#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "Car.h"
#include "util.h"

#define RIGHT_DIR 1
#define LEFT_DIR -1
#define LX 0.3 // Distance from center to wheels along x-axis (in meters)
#define LY 0.2  // Distance from center to wheels along y-axis (in meters)
#define WHEEL_RADIUS 0.05  // Radius of the wheel (in meters)


extern const int NUM_MOTORS;

// PID  Velocity constants
extern double kpVelo;
extern double kiVelo;
extern double kdVelo;

// PID Position constants
extern double kpPos;
extern double kiPos;
extern double kdPos;

extern int dir;
extern String message;
extern String message_car;

// Pin configurations for motors
extern motorPin motorPin0; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin1; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin2; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin3; // pinA, pinB, L_PWM, R_PWM

// Wheel objects
extern Wheel wheels[];

// Mecanum car object
extern Car mecanumCar;

// extern long time;

// Declare the trigger interrupt function
extern void triggerW0();
extern void triggerW1();
extern void triggerW2();
extern void triggerW3();

#endif // !__GLOBALSETTINGS_H__