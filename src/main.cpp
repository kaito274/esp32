#include "Motor.h" // #include "Server.h"
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
inline const char *ssid = "YOUR_SSID";         // Replace with your network credentials
inline const char *password = "YOUR_PASSWORD"; // Replace with your network credentials

#endif // Replace with your network credentials

#define UART_BUFFER_SIZE 256

HardwareSerial camSerial(2);
uint8_t buffer[UART_BUFFER_SIZE];
// JSON_UARTReader uart_reader(camSerial, 115200, 16, 17);
JsonDocument doc;

// Task handle for the send data task
TaskHandle_t sendDataTaskHandle;
void sendDataTask(void *parameter);
void mode_velocity();
void mode_position();
void uart_read();

void setup()
{
  Serial.begin(115200);
  camSerial.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(LED, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);

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

  // Set initial target position
  wheels[0].setTargetPosition(0);
  wheels[1].setTargetPosition(0);
  wheels[2].setTargetPosition(0);
  wheels[3].setTargetPosition(0);

  // Create the task for sending data, pinned to Core 1
  xTaskCreatePinnedToCore(
      sendDataTask,        // Task function
      "SendDataTask",      // Name of the task
      10000,               // Stack size (in words)
      NULL,                // Task parameter
      1,                   // Priority (higher value = higher priority)
      &sendDataTaskHandle, // Task handle
      1                    // Core to run the task (0 = Core 0, 1 = Core 1)
  );
}

void loop()
{

  current_millis = millis();

  // ============= CONTROL MODE ==============
  // VELOCITY MODE
  if (toggleMode == VELOCITY)
  {
    mode_velocity();
    digitalWrite(LED, LOW);
  }
  else
  { // POSITION MODE
    mode_position();
    digitalWrite(LED, HIGH);
  }

  uart_read();
}

void update_car_state(){
  if (current_millis - previous_millis_update_rpm <= interval_update_rpm) return;

  // Update car state 
  for (int i = 0; i < WHEEL_COUNT; i++) {
    wheels[i].updateRealRPM();
    wheels[i].resetEncValue();
  }
  mecanumCar.updateVelocity();
  mecanumCar.updatePosition();

  // Prepare data for plot
  mecanumCar.sendSocketCarPosition();
  for (int i = 0; i < WHEEL_COUNT; i++)
  {
    wheels[i].sendSocketVelocity();
  }
  previous_millis_update_rpm = current_millis;
}

void mode_velocity()
{
  update_car_state();

  // PID Tuning for velocity
  if (current_millis - previous_millis_pid_velocity > interval_pid_velocity)
  {
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
      wheels[i].tuningRPM();
      if (wheels[i].getDirection() == RIGHT_DIR)
      {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      }
      else
      {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      }
    }
    previous_millis_pid_velocity = current_millis;
  }

  mecanumCar.infoVelocity();
}

void mode_position()
{
  update_car_state();

  // PID Tuning for position
  if (current_millis - previous_millis_pid_position > interval_pid_position)
  {
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
      wheels[i].tuningPosition();
      if (wheels[i].getDirection() == LEFT_DIR)
      {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      }
      else
      {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      }
    }
    previous_millis_pid_position = current_millis;
  }
  
  if (current_millis - previous_millis_update_target_position > interval_update_target_position)
  {

    for (int i = 0; i < WHEEL_COUNT; i++)
    {
      wheels[i].setTargetPosition(positions[i]);
    }

    previous_millis_update_target_position = current_millis;
  }

  mecanumCar.infoPosition();
}

void uart_read()
{
  // DeserializationError error = uart_reader.read(doc);
  size_t bytes_read = 0;
  while (camSerial.available() > 0 && bytes_read < UART_BUFFER_SIZE - 1)
  {
    buffer[bytes_read] = camSerial.read();
    ++bytes_read;
  }

  buffer[bytes_read] = '\0';
  DeserializationError error = deserializeJson(doc, buffer);
  // if (error) {
  //   Serial.print(F("deserializeJson() failed: "));
  //   Serial.println(error.c_str());
  // } else {
  if (!error)
  {
    const operation_mode_t op_mode = doc["m"];
    switch (op_mode)
    {
    case (JOYSTICK_MANUAL):
    {
      toggleMode = VELOCITY;
      const double vx = doc["x"], vy = doc["y"];
      const uint8_t op_type = doc["t"];

      switch (op_type)
      {
      case (OMNIDIRECTIONAL):
      {
        // const uint8_t throttle = doc["th"];
        mecanumCar.move(vx * velo_test, vy * velo_test, 0);
        break;
      }
      case (ROTATIONAL):
      {
        const double wz = doc["z"];
        mecanumCar.move(0, 0, wz * velo_rotate);
        break;
      }
      default:
      {
        Serial.println("unknown movement type");
      }
      }

      break;
    }

    case (BUTTONS_MANUAL):
    {
      toggleMode = VELOCITY;
      const double vx = doc["x"], vy = doc["y"];
      const uint8_t throttle = doc["th"];

      Serial.print(vx);
      Serial.print(" ");
      Serial.print(vy);
      Serial.print(" ");
      Serial.print(throttle);
      Serial.println();

      // move(vx*0.2 , vy*0.2, 0);
      mecanumCar.move(vx * velo_test, vy * velo_test, 0);

      break;
    }

    case (BUTTONS_AUTO):
    {
      toggleMode = POSITION;

      // Update current positions
      positions[0] = wheels[0].getEncPosition();
      positions[1] = wheels[1].getEncPosition();
      positions[2] = wheels[2].getEncPosition();
      positions[3] = wheels[3].getEncPosition();

      // {"m":2,"step":1,"p1":"12.887","p2":"12.887","p3":"12.887","p4":"12.887"}
      const int p1 = doc["p1"], p2 = doc["p2"], p3 = doc["p3"], p4 = doc["p4"];
      const uint8_t step = doc["step"];
      // const double vx = doc["p1"], vy = doc["p2"];
      // const uint8_t throttle = doc["th"];

      Serial.println(step);
      Serial.println(p1);
      Serial.println(p2);
      Serial.println(p3);
      Serial.println(p4);

      positions[0] += p1;
      positions[1] += p2;
      positions[2] += p3;
      positions[3] += p4;

      break;
    }

    default:
    {
      Serial.println("unknown operaiton mode");
      break;
    }
    }
  }
}