#ifndef __UTIL_H__
#define __UTIL_H__

struct motorPin{
    int pinA, pinB, L_PWM, R_PWM;
    motorPin(int pinA, int pinB, int L_PWM, int R_PWM){
        this->pinA = pinA;
        this->pinB = pinB;
        this->L_PWM = L_PWM;
        this->R_PWM = R_PWM;
    }
};

#endif // __UTIL_H__