#ifndef __CAR_H__
#define __CAR_H__

#include "Wheel.h"

#define MAX_PWM 255 
#define MAX_RPM 333  

#define WHEEL_COUNT 4

class Car {
private:
    // Mecanum car dimensions 
    double lx; // Distance from center to wheels along x-axis (in meters)
    double ly; // Distance from center to wheels along y-axis (in meters)
    double r; // Radius of the wheel (in meters)

    double point_x; // x-coordinate of the car
    double point_y; // y-coordinate of the car
    double direction; // Direction of the car (360 degrees)
    long time = 0;
    Wheel** wheels;  

public:
    Car(double lx, double ly, double r, Wheel* wheels);
    void move(double vx, double vy, double wz);
    void updatePosition(double vx, double vy, double wz);
    ~Car(); 
};

#endif // __CAR_H__