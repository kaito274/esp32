#include "Car.h"
#include "GlobalSettings.h"

Car::Car(double lx, double ly, double r, Wheel* wheels): lx(lx), ly(ly), r(r), point_x(0), point_y(0), dir_angle(0)
{   
  this->wheels = new Wheel*[WHEEL_COUNT];
  for (int i = 0; i < WHEEL_COUNT; i++) {
    this->wheels[i] = &wheels[i];
  }
}

void Car::move(double vx, double vy, double wz)
{
  // vy *= -1; // Flip the sign of vy (Mecanum drive has inverted y-axis)
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

void Car::updatePosition() {
  long currentTime =  millis();
  double dt = (currentTime - time) / 1000.0; // Convert to seconds

  point_x = point_x + vxCar * dt;
  point_y = point_y + vyCar * dt;
  dir_angle = dir_angle + wzCar * dt * 180 / M_PI; // Convert to degrees
  dir_angle = fmod(dir_angle, 360);
  time = currentTime;
}

void Car::updateVelocity() {
  Wheel *w0 = wheels[0];
  Wheel *w1 = wheels[1];
  Wheel *w2 = wheels[2];
  Wheel *w3 = wheels[3];

  // Convert RPM to angular velocity (rad/s)
  double w_fl = w0->getCurrentRPM() * w0->getCurDirection() * 2 * M_PI / 60;
  double w_fr = -1 * w1->getCurrentRPM() * w1->getCurDirection() * 2 * M_PI / 60;
  double w_rl = w2->getCurrentRPM() * w2->getCurDirection() * 2 * M_PI / 60;
  double w_rr = -1 * w3->getCurrentRPM() * w3->getCurDirection() * 2 * M_PI / 60;

  // Calculate the linear and angular velocities of the car
  vxCar = r / 4 * (w_fl + w_fr + w_rl + w_rr);
  vyCar = r / 4 * (-w_fl + w_fr + w_rl - w_rr);
  wzCar = r / (4 * (lx + ly)) * (-w_fl + w_fr - w_rl + w_rr);

  Serial.print("w_fl: " + String(w_fl));
  Serial.print("\tw_fr: " + String(w_fr));
  Serial.print("\tw_rl: " + String(w_rl));
  Serial.print("\tw_rr: " + String(w_rr));
  Serial.print("\tvxCar: " + String(vxCar));
  Serial.print("\tvyCar: " + String(vyCar));
  Serial.print("\twzCar: " + String(wzCar));
  Serial.println("");

}

void Car::carInfo() {
  Serial.print("Point_x:" + String(point_x));
  Serial.print("\tPoint_y:" + String(point_y));
  Serial.print("\tDirection_Angle:" + String(dir_angle));
  Serial.println("");
  message_car = "Point_x:" + String(point_x) 
              + "\tPoint_y:" + String(point_y) 
              + "\tDirection_Angle:" + String(dir_angle);
}

Car::~Car()
{
  delete[] wheels;
}