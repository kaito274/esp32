#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "util.h"

extern const int NUM_MOTORS;

// PID constants
extern double kp;
extern double kd;
extern double ki;

// Pin configurations for motors
extern motorPin motorPin1; // pinA, pinB, EN, L_PWM, R_PWM

// Wheel objects
extern Wheel wheelsSpeed[];

// Declare the trigger interrupt function
extern void triggerW1A();
extern void triggerW1B();


#endif // !__GLOBALSETTINGS_H__