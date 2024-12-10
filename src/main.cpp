#include "Motor.h"
#include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

long previousEncoderValues = 0;


void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  int pinA = wheelsSpeed[0].getEncoder().getPinA();
  attachInterrupt(digitalPinToInterrupt(pinA), triggerW1A, RISING);
  wheelsSpeed[0].getMotor().Enable();
  wheelsSpeed[0].getMotor().TurnRight(50);

  initWiFi();
  startServer();
  
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    
    wheelsSpeed[0].tuningRPM();
    wheelsSpeed[0].info();
    wheelsSpeed[0].getMotor().TurnRight(wheelsSpeed[0].getPWM());
    previousMillis = currentMillis;
  }
}
