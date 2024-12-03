#ifndef __TEMP_H__
#define __TEMP_H__

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
// #include "BTS7960.h"

// Motor A pins
static int motor1Pin1 = 27; 
static int motor1Pin2 = 26; 
static int enable1Pin = 14; 


void initWiFi();
void moveForward();
void moveBackward();
void stopMotor();
void startServer();

#endif // __TEMP_H__