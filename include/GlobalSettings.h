#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "util.h"
#include <vector>

#define RIGHT_DIR 1
#define LEFT_DIR -1


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
extern std::vector<String> test_messages;

// Pin configurations for motors
extern motorPin motorPin0; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin1; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin2; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin3; // pinA, pinB, L_PWM, R_PWM

// Wheel objects
extern Wheel wheels[];

// Declare the trigger interrupt function
extern void triggerW0();
extern void triggerW1();
extern void triggerW2();
extern void triggerW3();

typedef enum {
    OMNIDIRECTIONAL,
    ROTATIONAL
} movement_t;

typedef enum {
    JOYSTICK_MANUAL,
    BUTTONS_MANUAL,
    BUTTONS_AUTO
} operation_mode_t;

#endif // !__GLOBALSETTINGS_H__