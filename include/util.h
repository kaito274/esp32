#ifndef __UTIL_H__
#define __UTIL_H__

struct motorPin{
    int EN, L_PWM, R_PWM;
    motorPin(int EN, int L_PWM, int R_PWM){
        this->EN = EN;
        this->L_PWM = L_PWM;
        this->R_PWM = R_PWM;
    }
};

#endif // __UTIL_H__