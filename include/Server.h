#ifndef __SERVER_H__
#define __SERVER_H__

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "Motor.h"
#include "config.h"
#include "GlobalSettings.h"

void initWiFi();
void moveForward();
void moveBackward();
void goStraight();
void stopMotor();
void startServer();

#endif 