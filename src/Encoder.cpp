#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB) : pinA(pinA), pinB(pinB), position(0), encoderValue(0) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
}

void Encoder::triggerA() {
    encoderValue++;
}

void Encoder::triggerB() {
    int B = digitalRead(pinB);
    if (B > 0) {
        position++;
    } else {
        position--;
    }
}

int Encoder::getPosition() {
    return position;
}

volatile long Encoder::getEncoderValue() {
    return encoderValue;
}

void Encoder::resetEncoderValue() {
    encoderValue = 0;
}

void Encoder::resetPosition() {
    position = 0;
}
