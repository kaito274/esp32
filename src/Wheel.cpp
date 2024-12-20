#include "Wheel.h"
#include "GlobalSettings.h"

Wheel::Wheel(){}

Wheel::Wheel(int id, int pinA, int pinB, int L_PWM, int R_PWM) {
    this->id = id;
    this->encoder = new Encoder(pinA, pinB);
    this->motor = new Motor(L_PWM, R_PWM);
    this->direction = 1;

    // PID Velocity initialization
    this->currentRPM = 0;
    this->targetRPM = 0;
    this->pidVelocity = new PID(&this->currentRPM, &this->computedPWMVelocity, &this->targetRPM, kpVelo, kiVelo, kdVelo, DIRECT); 
    this->pidVelocity->SetMode(AUTOMATIC);
    this->pidVelocity->SetOutputLimits(-255, 255);
    this->pidVelocity->SetSampleTime(500); // Set the sample time to 500ms

    // PID Position initialization
    this->currentPosition = 0;
    this->targetPosition = 0;
    this->pidPosition = new PID(&this->currentPosition, &this->computedPWMPosition, &this->targetPosition, kpPos, kiPos, kdPos, DIRECT); 
    this->pidPosition->SetMode(AUTOMATIC);
    this->pidPosition->SetOutputLimits(-255, 255);

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

void Wheel::resetEncValue() {
    this->encoder->resetEncoderValue();
}

int Wheel::getPWM() {
    return this->pwm;
}

int Wheel::getDirection() {
    return this->direction;
}

void Wheel::triggerInterrupt() {
    encoder->triggerInterrupt();
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

void Wheel::setDirection(int direction) {
    assert(direction == LEFT_DIR || direction == RIGHT_DIR); // Ensure the direction is either 1 (turn left) or -1 (turn right)
    this->direction = direction;
}

void Wheel::tuningRPM() {
    double currentRPM = (float)this->getEncValue() * 60 / ENC_COUNT_REV;
    this->currentRPM = currentRPM;
    this->pidVelocity->SetTunings(kpVelo, kiVelo, kdVelo);
    this->pidVelocity->Compute(); // Compute the PID output
    this->pwm += this->computedPWMVelocity;
    this->pwm = constrain(this->pwm, 0, 255);
    // this->encoder->resetEncoderValue();
}

void Wheel::tuningPosition() {

    // noInterrupts(); // TODO: Check if this is necessary
    int currentPos = this->getEncPosition();
    this->currentPosition = currentPos;     
    // interrupts();
    this->pidPosition->Compute(); // Compute the PID output
    double error = targetPosition - currentPosition;

    // Apply the PID output to the motor (convert to PWM)
    this->pwm = constrain((int)fabs(error), 0, 100); 

    // Determine the direction (sign of the output)
    this->direction = (error > 0) ? RIGHT_DIR : LEFT_DIR; //TODO: Check if this is correct
}

String Wheel::infoVelocity() {
    double currentRPM = (float)this->getEncValue() * 60 / ENC_COUNT_REV;

    // Serial.println("######### Wheel Velocity Info #########");
    Serial.print("Wheel_ID:" + String(this->id));
    Serial.print("\tTarget_RPM:"); Serial.print(this->getTargetRPM());
    Serial.print("\tPulse:"); Serial.print(this->getEncValue());
    Serial.print("\tCurrent_RPM:"); Serial.print(currentRPM);
    Serial.print("\tDirection:"); Serial.print(this->direction == RIGHT_DIR ? "RIGHT" : "LEFT");
    Serial.print("\tPWM:"); Serial.println(this->pwm);
    // Serial.print("\tERROR:"); Serial.println(this->computedPWMVelocity);
    String test = "Target_RPM:" + String(this->getTargetRPM()) 
                + "\tCurrent_RPM:" + String(this->currentRPM) 
                + "\tPWM:" + String(this->pwm) 
                + "\tERROR:" + String(this->computedPWMVelocity);
    return test;
}

void Wheel::infoPosition() {
    // Serial.println("######### Wheel Position Info #########");
    Serial.print("Wheel_ID:" + String(this->id));
    Serial.print("\tTarget_position:"); Serial.print(this->getTargetPosition());
    Serial.print("\tCurrent_position:"); Serial.print(this->getEncPosition());
    Serial.print("\tDirection:"); Serial.print(this->direction == RIGHT_DIR ? "RIGHT" : "LEFT");
    Serial.print("\tMotor_Power:"); Serial.print(this->pwm);
    Serial.print("\tError:"); Serial.println(targetPosition - currentPosition);
    // message = "Target_position:" + String(this->getTargetPosition()) 
    //         + "\tCurrent_position:" + String(this->getEncPosition()) 
    //         + "\tDirection:" + String(this->direction) 
    //         + "\tMotor_Power:" + String(this->pwm) 
    //         + "\tError:" + String(targetPosition - currentPosition) + "\n";
}

void Wheel::infoPin() {
    // // Serial.println("######### Pin Info #########");
    // Serial.print("pinA: "); Serial.print(this->encoder->getPinA()); 
    // Serial.print("\tpinB: "); Serial.print(this->encoder->getPinB());
    // Serial.print("\tL_PWM: "); Serial.print(this->motor->_L_PWM);
    // Serial.print("\tR_PWM: "); Serial.println(this->motor->_R_PWM);
}

Wheel::~Wheel() {
    delete this->encoder;
    delete this->motor;
    delete this->pidVelocity;
    delete this->pidPosition;
}