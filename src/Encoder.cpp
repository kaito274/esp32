#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB) : pinA(pinA), pinB(pinB), position(0), encoderValue(0) {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
}

void Encoder::triggerA() {
    encoderValue += 1;
    int B = digitalRead(pinB);
    if (B > 0) {
        position++;
    } else {
        position--;
    }
}

void Encoder::triggerB() {
    int B = digitalRead(pinB);
    if (B > 0) {
        position++;
    } else {
        position--;
    }
}

volatile long Encoder::getPosition() {
    return position;
}

volatile long Encoder::getEncoderValue() {
    return encoderValue;
}

int Encoder::getPinA() {
    return pinA;
}

int Encoder::getPinB() {
    return pinB;
}

void Encoder::resetEncoderValue() {
    encoderValue = 0;
}

void Encoder::resetPosition() {
    position = 0;
}
