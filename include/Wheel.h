#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "Motor.h"
#include "Encoder.h"
#include <PID_v1.h>

class Wheel {
private:
    PID *pidVelocity;
    PID *pidPosition;
    Encoder *encoder;
    Motor *motor;

    // PID Velocity constants
    double currentRPM;  
    double computedPWMVelocity; 
    double targetRPM;

    // PID Position constants
    double currentPosition;
    double computedPWMPosition;
    double targetPosition;

    double pwm = 0;
public:
    Wheel();
    Wheel(int pinA, int pinB, int L_PWM, int R_PWM);
    Wheel(int pinA, int pinB, int L_PWM, int R_PWM, double currentRPM, double targetRPM, double Kp, double Ki, double Kd);
    // void initPIDVelocity(double currentRPM, double targetRPM, double Kp, double Ki, double Kd);
    void initEncoder();
    void initMotor();
    PID getPIDVelocity();
    PID getPIDPosition();
    Encoder getEncoder();
    Motor getMotor();

    // PID Velocity functions
    double getCurrentRPM();
    double getComputedPWMVelocity();
    double getTargetRPM();
    void setCurrentRPM(double currentRPM);
    void setTargetRPM(double targetRPM);

    // PID Position functions
    double getCurrentPosition();
    double getComputedPWMPosition();
    double getTargetPosition();
    void setCurrentPosition(double currentPosition);
    void setTargetPosition(double targetPosition);

    volatile long getEncValue();
    volatile long getEncPosition();
    int getEncPinA();
    int getEncPinB();
    int getPosition();
    int getPWM();

    void triggerA();
    void triggerB();
    void setPWM(double pwm);
    void tuningRPM();
    void info();

    ~Wheel();
};

#endif // !__WHEEL_H__
