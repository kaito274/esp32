#include "Wheel.h"

Wheel::Wheel(){}

Wheel::Wheel(int pinA, int pinB, int EN, int L_PWM, int R_PWM, double input = 0, double setpoint = 0, double Kp = 1.0, double Ki = 0.0, double Kd = 0.0) 
    : input(input), setpoint(setpoint) {
    this->encoder = new Encoder(pinA, pinB);
    this->motor = new Motor(EN, L_PWM, R_PWM);
    this->pid = new PID(&this->input, &this->output, &this->setpoint, Kp, Ki, Kd, DIRECT);

    this->pid->SetMode(AUTOMATIC);
    this->pid->SetOutputLimits(-255, 255);
} 

void Wheel::initPID(double input, double setpoint, double Kp, double Ki, double Kd) {
    this->input = input;
    this->setpoint = setpoint;
    this->pid = new PID(&this->input, &this->output, &setpoint, Kp, Ki, Kd, DIRECT);
    this->pid->SetMode(AUTOMATIC);
    this->pid->SetOutputLimits(-255, 255);
}


PID Wheel::getPID() {
    return *this->pid;
}

Encoder Wheel::getEncoder() {
    return *this->encoder;
}

Motor Wheel::getMotor() {
    return *this->motor;
}

void Wheel::setSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

Wheel::~Wheel() {
    delete this->encoder;
    delete this->motor;
    delete this->pid;
}