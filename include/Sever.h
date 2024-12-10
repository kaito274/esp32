#ifndef __SEVER_H__
#define __SEVER_H__

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// #include "Motor.h"

// Motor A pins
static int motor1Pin1 = 27; 
static int motor1Pin2 = 26; 
static int enable1Pin = 14; 


void initWiFi();
void moveForward();
void moveBackward();
void stopMotor();
void startServer();

#endif 