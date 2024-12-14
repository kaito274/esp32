#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "util.h"

extern const int NUM_MOTORS;

// PID constants
extern double kp;
extern double kd;
extern double ki;

extern int dir;

// Pin configurations for motors
extern motorPin motorPin1; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin2; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin3; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin4; // pinA, pinB, L_PWM, R_PWM

// Wheel objects
extern Wheel wheelsSpeed[];

// Declare the trigger interrupt function
extern void triggerW1A();
extern void triggerW1B();
extern void triggerW2A();
extern void triggerW2B();
extern void triggerW3A();
extern void triggerW3B();
extern void triggerW4A();
extern void triggerW4B();

#endif // !__GLOBALSETTINGS_H__