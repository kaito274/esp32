#include "Motor.h"
// #include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
#include "Car.h"
#include "config.h"
#include <WiFi.h>
#include <PID_v1.h>

#define LED 2

// Server settings
#ifndef __CONFIG_H__
#define __CONFIG_H__
inline const char *ssid = "YOUR_SSID"; // Replace with your network credentials
inline const char *password = "YOUR_PASSWORD"; // Replace with your network credentials

#endif// Replace with your network credentials

#define VELOCITY 0
// #define POSITION 1


// interval for measurements
int interval_velocity = 1000;
int interval_position = 1;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

const int port = 8080;
const int stopSignal = 20;
WiFiServer server(port);

// Flag to track client status
bool isClientConnected = false;

// Task handle for the send data task
TaskHandle_t sendDataTaskHandle;

void sendDataTask(void *parameter)
{
  Serial.println("Core 1");
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();

  while (1)
  {
    WiFiClient client = server.available();
    if (client)
    {
      while (client.connected())
      {
        Serial.println(message);
        client.print(message);
        delay(50);
      }
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  // Attach the interrupt functions to the encoder pins
  attachInterrupt(digitalPinToInterrupt(wheels[0].getEncPinA()), triggerW0, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[1].getEncPinA()), triggerW1, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[2].getEncPinA()), triggerW2, RISING);
  attachInterrupt(digitalPinToInterrupt(wheels[3].getEncPinA()), triggerW3, RISING);

  // Enable the motor
  wheels[0].getMotor().Enable();
  wheels[1].getMotor().Enable();
  wheels[2].getMotor().Enable();
  wheels[3].getMotor().Enable();

#ifdef VELOCITY
  // wheels[0].setTargetRPM(50);
  // wheels[1].setTargetRPM(75);
  // wheels[2].setTargetRPM(100);
  // wheels[3].setTargetRPM(125);
#endif // VELOCITY

#ifdef POSITION
  wheels[0].setTargetPosition(3000);
#endif // POSITION


  // TODO: Testing PID plot
    // Create the task for sending data, pinned to Core 1
  // xTaskCreatePinnedToCore(
  //   sendDataTask,    // Task function
  //   "SendDataTask",  // Name of the task
  //   10000,           // Stack size (in words)
  //   NULL,            // Task parameter
  //   5,               // Priority (higher value = higher priority)5
  //   &sendDataTaskHandle, // Task handle
  //   1                // Core to run the task (0 = Core 0, 1 = Core 1)
  // );

  // TODO: ???
  // Serial.begin(115200);
  // WiFi.begin(ssid_, password_);

  // // Wait for connection
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  // Serial.println("Connected to WiFi!");
  // Serial.print("ESP32 IP Address: ");
  // Serial.println(WiFi.localIP());

  // // Start the serverd
  // server.begin();

}

void SerialDataWrite();
void command_test(char option);

void loop()
{
  // WiFiClient client = server.available();

#ifdef VELOCITY
  // Velocity
  currentMillis = millis();

  if (currentMillis - previousMillis > interval_velocity) {
    for(int i = 0; i < WHEEL_COUNT; i++){
      wheels[i].tuningRPM();
      
      if (wheels[i].getDirection() == RIGHT_DIR) {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      } else {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      }
      wheels[i].infoVelocity();
      wheels[i].resetEncValue(); // Reset encoder value
    }
    mecanumCar.updateVelocity();
    mecanumCar.updatePosition();
    mecanumCar.carInfo();
    previousMillis = currentMillis;
  }
  
  SerialDataWrite();
#endif // VELOCITY

#ifdef POSITION
  // Position
  currentMillis = millis();
  if (currentMillis - previousMillis > interval_position)
  {
    for (int i = 0; i < WHEEL_COUNT; i++) {
      wheels[i].infoPosition();
      wheels[i].tuningPosition();
      if (wheels[i].getDirection() == LEFT_DIR) {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      } else {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      }
    }
    previousMillis = currentMillis;
  }

#endif // POSITION

}

void SerialDataWrite()
{
  static String received_chars;
  while (Serial.available())
  {
    char inChar = (char)Serial.read();
    received_chars += inChar;
    if (inChar == '\n')
    {
      // Serial.println(received_chars);
      // // wheels[0].setCurrentRPM(received_chars.toInt());
      // // input = received_chars.toInt();
      //  wheels[0].setTargetPosition(received_chars.toInt());
      // received_chars = "";
      int option = received_chars.toInt();
      // Serial.println(option);
      // Serial.println("2137123");
      char c = received_chars[0];
      command_test(c);
      received_chars = "";
    }
  }
}
double velo_test = 0.20;
double velo_rotate = 0.25;

void command_test(char option){
  Serial.print("Change movement: ");
  Serial.println(option);
  switch (option)
  {
  case '1':
    mecanumCar.move(-velo_test, velo_test, 0.0);
    break;
  case '2':
    mecanumCar.move(-velo_test, 0, 0.0);
    break;
  case '3':
    mecanumCar.move(-velo_test, -velo_test, 0.0);
    break;
  case '4':
    mecanumCar.move(0, velo_test, 0.0);
    break;
  case '5':
    mecanumCar.move(0.0, 0.0, 0.0);
    Serial.println("Stopped");
    break;
  case '6':
    mecanumCar.move(0, -velo_test, 0.0);
    break;
  case '7':
    mecanumCar.move(velo_test, velo_test, 0.0);
    break;
  case '8':
    mecanumCar.move(velo_test, 0, 0.0);
    break;
  case '9':
    mecanumCar.move(velo_test, -velo_test, 0.0);
    break;
  case 'p': //rotate right
    mecanumCar.move(0, 0, -velo_rotate);
    break;
  case 'o': //rotate left
    mecanumCar.move(0, 0, velo_rotate);
    break;
  case 'k': //drift left
    mecanumCar.move(0.0, velo_test, velo_rotate);
    break;
  case 'l': //drift rightp
    mecanumCar.move(0.0, -velo_test, -velo_rotate);
    break;
  default:
    mecanumCar.move(0, 0, 0);
    Serial.println("Error");
    break;
  }
}