#include "Car.h"

Car::Car(double lx, double ly, double r, Wheel* wheels): lx(lx), ly(ly), r(r), point_x(0), point_y(0), direction(0)
{   
  this->wheels = new Wheel*[WHEEL_COUNT];
  for (int i = 0; i < WHEEL_COUNT; i++) {
    this->wheels[i] = &wheels[i];
  }
}

void Car::move(double vx, double vy, double wz)
{
  vy *= -1; // Flip the sign of vy (Mecanum drive has inverted y-axis)
  double pwmFL, pwmFR, pwmRL, pwmRR;

  // Calculate wheel angular velocities in rad/s
  double w_fl = (1 / r) * (vx - vy - (lx + ly) * wz);
  double w_fr = -(1 / r) * (vx + vy + (lx + ly) * wz); // Flip direction
  double w_rl = (1 / r) * (vx + vy - (lx + ly) * wz);
  double w_rr = -(1 / r) * (vx - vy + (lx + ly) * wz); // Flip direction

  // Convert angular velocities to RPM
  double rpmFL = w_fl * 60 / (2 * M_PI);
  double rpmFR = w_fr * 60 / (2 * M_PI);
  double rpmRL = w_rl * 60 / (2 * M_PI);
  double rpmRR = w_rr * 60 / (2 * M_PI);

  // Cap RPM values to MAX_RPM
  rpmFL = constrain(rpmFL, -MAX_RPM, MAX_RPM);
  rpmFR = constrain(rpmFR, -MAX_RPM, MAX_RPM);
  rpmRL = constrain(rpmRL, -MAX_RPM, MAX_RPM);
  rpmRR = constrain(rpmRR, -MAX_RPM, MAX_RPM);

  // Set direction of the wheels
  int dirFL = (w_fl > 0) ? 1 : -1;
  int dirFR = (w_fr > 0) ? 1 : -1;
  int dirRL = (w_rl > 0) ? 1 : -1;
  int dirRR = (w_rr > 0) ? 1 : -1;

  // // Convert RPMs to PWM values (absolute values)
  // pwmFL = map(abs(rpmFL), 0, MAX_RPM, 0, MAX_PWM);
  // pwmFR = map(abs(rpmFR), 0, MAX_RPM, 0, MAX_PWM);
  // pwmRL = map(abs(rpmRL), 0, MAX_RPM, 0, MAX_PWM);
  // pwmRR = map(abs(rpmRR), 0, MAX_RPM, 0, MAX_PWM);

  // // Apply the computed PWM values to the motors
  // wheels[0].setPWM(pwmFL);
  // wheels[1].setPWM(pwmFR);
  // wheels[2].setPWM(pwmRL);
  // wheels[3].setPWM(pwmRR);

  // TODO: Using PIDvelo_rotate
  wheels[0]->setTargetRPM(abs(rpmFL));
  wheels[1]->setTargetRPM(abs(rpmFR));
  wheels[2]->setTargetRPM(abs(rpmRL));
  wheels[3]->setTargetRPM(abs(rpmRR));

  // Set the direction of the wheels
  wheels[0]->setDirection(dirFL);
  wheels[1]->setDirection(dirFR);
  wheels[2]->setDirection(dirRL);
  wheels[3]->setDirection(dirRR);
}

void Car::updatePosition(double vx, double vy, double wz) {
  long currentTime =  millis();
  double dt = (currentTime - time) / 1000.0; // Convert to seconds
  point_x += vx * dt;
  point_y += vy * dt;
  direction += wz * dt;
  direction = fmod(direction, 360); // Ensure the direction is within [0, 360] degrees
}

Car::~Car()
{
  delete[] wheels;
}