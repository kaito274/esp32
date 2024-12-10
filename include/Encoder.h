#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <Arduino.h>

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

class Encoder {
private: 
    int pinA, pinB;
    volatile long encoderValue;
    int position;

    // Static interrupt handler functions
    // static void staticTriggerA();
    // static void staticTriggerB();

public:
    Encoder(int pinA, int pinB);
    void triggerA();
    void triggerB();
    int getPosition();
    int getPinA();
    int getPinB();
    volatile long getEncoderValue();
    void resetEncoderValue();
    void resetPosition();

};

#endif