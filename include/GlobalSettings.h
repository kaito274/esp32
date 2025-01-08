#ifndef __GLOBALSETTINGS_H__
#define __GLOBALSETTINGS_H__

#include "Wheel.h"
#include "Car.h"
#include "util.h"
#include <vector>

#define RIGHT_DIR 1
#define LEFT_DIR -1
#define LX 0.25 // Distance from center to wheels along x-axis (in meters)
#define LY 0.25  // Distance from center to wheels along y-axis (in meters)
#define WHEEL_RADIUS 0.05  // Radius of the wheel (in meters)

#define VELOCITY 0
#define POSITION 1

#define MILLIS_PER_MINUTE 60000

extern const int NUM_MOTORS;

extern int toggleMode;

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
extern String message_car;

// interval for measurements
extern int interval_velocity;
extern int interval_position;
extern int interval_pid_velocity;
extern int interval_pid_position;
extern int interval_velocity_info;
extern int interval_position_info;
extern int interval_car_info;
extern int interval_update_rpm;
extern int interval_update_target_position;

extern long previous_millis_info_velocity;
extern long previous_millis_pid_velocity;
extern long previous_millis_pid_position;
extern long previous_millis_info_position;
extern long previous_millis_car_info;
extern long previous_millis_update_rpm;
extern long previous_millis_update_target_position;

extern long current_millis;



// Pin configurations for motors
extern motorPin motorPin0; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin1; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin2; // pinA, pinB, L_PWM, R_PWM
extern motorPin motorPin3; // pinA, pinB, L_PWM, R_PWM

// Wheel objects
extern Wheel wheels[];

// Mecanum car object
extern Car mecanumCar;



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