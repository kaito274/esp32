#include "Motor.h"
// #include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
// #include "JSON_UARTReader.h"
#include "Car.h"
#include "config.h"
#include <WiFi.h>
#include <PID_v1.h>
#include <ArduinoJson.h>

#define LED 2

// Server settings
#ifndef __CONFIG_H__
#define __CONFIG_H__
inline const char *ssid = "YOUR_SSID"; // Replace with your network credentials
inline const char *password = "YOUR_PASSWORD"; // Replace with your network credentials

#endif// Replace with your network credentials

#define VELOCITY 0
// #define POSITION 1

HardwareSerial camSerial(2);
uint8_t buffer[UART_BUFFER_SIZE];
// JSON_UARTReader uart_reader(camSerial, 115200, 16, 17);
JsonDocument doc;

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
        // Serial.println(message);
        client.println(message_car);
        delay(250);
      }
      // delay(150);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  camSerial.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(LED, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);


  // move(0.0, 0.0, 1.0);

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
  xTaskCreatePinnedToCore(
    sendDataTask,    // Task function
    "SendDataTask",  // Name of the task
    10000,           // Stack size (in words)
    NULL,            // Task parameter
    10,               // Priority (higher value = higher priority)5
    &sendDataTaskHandle, // Task handle
    1                // Core to run the task (0 = Core 0, 1 = Core 1)
  );

  // TODO: ???
  Serial.begin(115200);
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

  // Start the serverd
  server.begin();




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

  // DeserializationError error = uart_reader.read(doc);
  size_t bytes_read = 0;
  while (camSerial.available() > 0 && bytes_read < UART_BUFFER_SIZE - 1) {
    buffer[bytes_read] = camSerial.read();
    ++bytes_read;
  }

  buffer[bytes_read] = '\0';
  DeserializationError error = deserializeJson(doc, buffer);
  // if (error) {
  //   Serial.print(F("deserializeJson() failed: "));
  //   Serial.println(error.c_str());
  // } else {
  if (!error) {
    const movement_t mv_type = doc["t"];

    switch (mv_type) {
      case (OMNIDIRECTIONAL): {
        const operation_mode_t op_type = doc["m"];

        switch (op_type) {
          case (BUTTONS_MANUAL): {
            const double vx = doc["x"], vy = doc["y"];
            const uint8_t throttle = doc["th"];

            Serial.print(vx);
            Serial.print(" ");
            Serial.print(vy);
            Serial.print(" ");
            Serial.print(throttle);
            Serial.println();

            // move(vx*0.2 , vy*0.2, 0);
            move(vy*0.2, -vx*0.2, 0);

            break;
          }

          default: {
            Serial.println("unknown operaiton mode");
          }
        }

        break;
      }
      case (ROTATIONAL): {
        
        break;
      }
      default: {
        Serial.println("unknown movement type");
      }
    }
  }

  // DeserializationError error = uart_reader.read(doc);
  size_t bytes_read = 0;
  while (camSerial.available() > 0 && bytes_read < UART_BUFFER_SIZE - 1) {
    buffer[bytes_read] = camSerial.read();
    ++bytes_read;
  }

  buffer[bytes_read] = '\0';
  DeserializationError error = deserializeJson(doc, buffer);
  // if (error) {
  //   Serial.print(F("deserializeJson() failed: "));
  //   Serial.println(error.c_str());
  // } else {
  if (!error) {
    const movement_t mv_type = doc["t"];

    switch (mv_type) {
      case (OMNIDIRECTIONAL): {
        const operation_mode_t op_type = doc["m"];

        switch (op_type) {
          case (BUTTONS_MANUAL): {
            const double vx = doc["x"], vy = doc["y"];
            const uint8_t throttle = doc["th"];

            Serial.print(vx);
            Serial.print(" ");
            Serial.print(vy);
            Serial.print(" ");
            Serial.print(throttle);
            Serial.println();

            // move(vx*0.2 , vy*0.2, 0);
            move(vy*0.2, -vx*0.2, 0);

            break;
          }

          default: {
            Serial.println("unknown operaiton mode");
          }
        }

        break;
      }
      case (ROTATIONAL): {
        
        break;
      }
      default: {
        Serial.println("unknown movement type");
      }
    }
  }

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

#define MAX_RPM 333  // Replace with your motor's max RPM
#define MAX_PWM 255  // Replace with your motor's max PWM value



void move(double vx, double vy, double wz)
{
  vy *= -1; // Flip the sign of vy (Mecanum drive has inverted y-axis)
  double pwmFL, pwmFR, pwmRL, pwmRR;

  // Mecanum car dimensions (example values, adjust as needed)
  double lx = 0.3; // Distance from center to wheels along x-axis (in meters)
  double ly = 0.2; // Distance from center to wheels along y-axis (in meters)
  double r = 0.05; // Radius of the wheel (in meters)

  // Calculate wheel angular velocities in rad/s
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
  wheels[0].setTargetRPM(abs(rpmFL));
  wheels[1].setTargetRPM(abs(rpmFR));
  wheels[2].setTargetRPM(abs(rpmRL));
  wheels[3].setTargetRPM(abs(rpmRR));

  // Set the direction of the wheels
  wheels[0].setDirection(dirFL);
  wheels[1].setDirection(dirFR);
  wheels[2].setDirection(dirRL);
  wheels[3].setDirection(dirRR);
}