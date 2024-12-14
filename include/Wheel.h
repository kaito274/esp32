#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "Motor.h"
#include "Encoder.h"
#include <PID_v1.h>

class Wheel {
private:
    PID *pid;
    Encoder *encoder;
    Motor *motor;
    double input;  // Current position (input for PID)
    double output; // PID output
    double setpoint; // target position for the wheel
    double pwm = 0;
public:
    Wheel();
    Wheel(int pinA, int pinB, int L_PWM, int R_PWM, double input, double setpoint, double Kp, double Ki, double Kd);
    void initPID(double input, double setpoint, double Kp, double Ki, double Kd);
    void initEncoder();
    void initMotor();
    PID getPID();
    Encoder getEncoder();
    Motor getMotor();
    int getInput();
    int getOutput();
    int getSetpoint();
    volatile long getEncoderValue();
    int getPosition();
    int getPWM();

    void triggerA();
    void triggerB();
    void setInput(double input);
    void setSetpoint(double setpoint);
    void setPWM(double pwm);
    void tuningRPM();
    void info();

    ~Wheel();
};

#endif // !__WHEEL_H__
