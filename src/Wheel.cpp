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

int Wheel::getInput() {
    return this->input;
}

int Wheel::getOutput() {
    return this->output;
}

int Wheel::getSetpoint() {
    return this->setpoint;
}

volatile long Wheel::getEncoderValue() {
    return this->encoder->getEncoderValue();
}

int Wheel::getPosition() {
    return this->encoder->getPosition();
}

int Wheel::getPWM() {
    return this->pwm;
}

void Wheel::triggerA() {
    encoder->triggerA();
}

void Wheel::triggerB() {
    encoder->triggerB();
}

void Wheel::setInput(double input) {
    this->input = input;
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

void Wheel::setPWM(double pwm) {
    this->pwm = pwm;
}

void Wheel::tuningRPM(double current_rpm) {
    this->input = current_rpm;
    this->pid->Compute();
    this->pwm += this->output;
    this->pwm = constrain(this->pwm, 0, 255);
    this->encoder->resetEncoderValue();
}

void Wheel::info() {
    Serial.println("######### Wheel Info #########");
    Serial.print("PWM: ");
    Serial.print(this->pwm);
    Serial.print("\tPULSES/Encoder Values: ");
    Serial.print(this->getEncoderValue());
    Serial.print("\tRPM: ");
    Serial.print(this->input);
    Serial.print("\tERROR: ");
    Serial.println(this->output);
}

Wheel::~Wheel() {
    delete this->encoder;
    delete this->motor;
    delete this->pid;
}