#include "Wheel.h"
#include "GlobalSettings.h"

Wheel::Wheel(){}

Wheel::Wheel(int pinA, int pinB, int L_PWM, int R_PWM) {
    this->encoder = new Encoder(pinA, pinB);
    this->motor = new Motor(L_PWM, R_PWM);

    // PID Velocity initialization
    this->currentRPM = 0;
    this->targetRPM = 0;
    this->pidVelocity = new PID(&this->currentRPM, &this->computedPWMVelocity, &this->targetRPM, kpVelo, kiVelo, kdVelo, DIRECT); // TODO: Change to PID constants
    this->pidVelocity->SetMode(AUTOMATIC);
    this->pidVelocity->SetOutputLimits(-255, 255);

    // PID Position initialization
    this->currentPosition = 0;
    this->targetPosition = 0;
    this->pidPosition = new PID(&this->currentPosition, &this->computedPWMPosition, &this->targetPosition, kpPos, kiPos, kdPos, DIRECT); // TODO: Change to PID constants
    this->pidPosition->SetMode(AUTOMATIC);
    this->pidPosition->SetOutputLimits(-255, 255);

}

Wheel::Wheel(int pinA, int pinB, int L_PWM, int R_PWM, double currentRPM = 0, double targetRPM = 0, double Kp = 1.0, double Ki = 0.0, double Kd = 0.0) 
    : currentRPM(currentRPM), targetRPM(targetRPM) {
    this->encoder = new Encoder(pinA, pinB);
    this->motor = new Motor(L_PWM, R_PWM);

    this->pidVelocity = new PID(&this->currentRPM, &this->computedPWMVelocity, &this->targetRPM, Kp, Ki, Kd, DIRECT);
    this->pidVelocity->SetMode(AUTOMATIC);
    this->pidVelocity->SetOutputLimits(-255, 255);
} 

// void Wheel::initPIDVelocity(double currentRPM, double targetRPM, double Kp, double Ki, double Kd) {
//     this->currentRPM = currentRPM;
//     this->targetRPM = targetRPM;
//     this->pid = new PID(&this->currentRPM, &this->computedPWMVelocity, &this->targetRPM, Kp, Ki, Kd, DIRECT);
//     this->pid->SetMode(AUTOMATIC);
//     this->pid->SetOutputLimits(-255, 255);
// }

PID Wheel::getPIDVelocity() {
    return *this->pidVelocity;
}

PID Wheel::getPIDPosition() {
    return *this->pidPosition;
}

// START: PID Velocity functions 
double Wheel::getCurrentRPM() {
    return this->currentRPM;
}

double Wheel::getComputedPWMVelocity() {
    return this->computedPWMVelocity;
}

double Wheel::getTargetRPM() {
    return this->targetRPM;
}

void Wheel::setTargetRPM(double targetRPM) {
    this->targetRPM = targetRPM;
}

void Wheel::setCurrentRPM(double currentRPM) {
    this->currentRPM = currentRPM;
}
// END: PID Velocity functions

// START: PID Position functions
double Wheel::getCurrentPosition() {
    return this->currentPosition;
}

double Wheel::getComputedPWMPosition() {
    return this->computedPWMPosition;
}

double Wheel::getTargetPosition() {
    return this->targetPosition;
}

void Wheel::setCurrentPosition(double currentPosition) {
    this->currentPosition = currentPosition;
}

void Wheel::setTargetPosition(double targetPosition) {
    this->targetPosition = targetPosition;
}
// END PID Position functions

volatile long Wheel::getEncValue() {
    return this->encoder->getEncoderValue();
}

volatile long Wheel::getEncPosition() {
    return this->encoder->getPosition();
}

int Wheel::getEncPinA() {
    return this->encoder->getPinA();
}

int Wheel::getEncPinB() {
    return this->encoder->getPinB();
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

Encoder Wheel::getEncoder() {
    return *this->encoder;
}

Motor Wheel::getMotor() {
    return *this->motor;
}

void Wheel::setPWM(double pwm) {
    this->pwm = pwm;
}

void Wheel::tuningRPM() {
    double currentRPM = (float)this->getEncValue() * 60 / ENC_COUNT_REV;
    this->currentRPM = currentRPM;
    this->pidVelocity->Compute();
    this->pwm += this->computedPWMVelocity;
    this->pwm = constrain(this->pwm, 0, 255);
    this->encoder->resetEncoderValue();
}

void Wheel::info() {
    Serial.println("######### Wheel Info #########");
    Serial.print("PWM: ");
    Serial.print(this->pwm);
    Serial.print("\tPULSES/Encoder Values: ");
    Serial.print(this->getEncValue());
    Serial.print("\tRPM: ");
    Serial.print(this->currentRPM);
    Serial.print("\tERROR: ");
    Serial.println(this->computedPWMVelocity);
}

Wheel::~Wheel() {
    delete this->encoder;
    delete this->motor;
    delete this->pidVelocity;
    delete this->pidPosition;
}