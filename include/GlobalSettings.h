#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "util.h"

#define LEFT_DIR 1
#define RIGHT_DIR -1

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

// Pin configurations for motors
extern motorPin motorPin1; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin2; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin3; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin4; // pinA, pinB, L_PWM, R_PWM

// Wheel objects
extern Wheel wheels[];

// Declare the trigger interrupt function
extern void triggerW1();
extern void triggerW2();
extern void triggerW3();
extern void triggerW4();

#endif // !__GLOBALSETTINGS_H__