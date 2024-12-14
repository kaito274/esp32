#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <Arduino.h>

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

class Encoder
{
private:
    int pinA, pinB;
    volatile long encoderValue;
    volatile long position;

public:
    Encoder(int pinA, int pinB);
    void triggerInterrupt();
    volatile long getPosition();
    volatile long getEncoderValue();
    int getPinA();
    int getPinB();
    void resetEncoderValue();
    void resetPosition();
};

#endif