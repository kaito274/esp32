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
inline const char *ssid = "YOUR_SSID"; // Replace with your network credentials
inline const char *password = "YOUR_PASSWORD"; // Replace with your network credentials

#endif// Replace with your network credentials

#define UART_BUFFER_SIZE 256
// #define VELOCITY 0
#define POSITION 1

HardwareSerial camSerial(2);
uint8_t buffer[UART_BUFFER_SIZE];
// JSON_UARTReader uart_reader(camSerial, 115200, 16, 17);
JsonDocument doc;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

double velo_test = 0.375;
double velo_rotate = 0.25;

long previousMillisInfoVelocity = 0;
long previousMillisPIDPosition = 0;

int interval_pid_position = 1000;
long previousMillisInfoPosition = 0;
int positions[] = {0, 0, 0, 0};

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
        for(int i = 0; i < WHEEL_COUNT; i++){
          client.println(test_messages[i]);
          delay(250);
        }
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
  // wheels[0].setTargetPosition(300);
#endif // POSITION


  // // TODO: Testing PID plot
  //   // Create the task for sending data, pinned to Core 1
  // xTaskCreatePinnedToCore(
  //   sendDataTask,    // Task function
  //   "SendDataTask",  // Name of the task
  //   10000,           // Stack size (in words)
  //   NULL,            // Task parameter
  //   10,               // Priority (higher value = higher priority)5
  //   &sendDataTaskHandle, // Task handle
  //   1                // Core to run the task (0 = Core 0, 1 = Core 1)
  // );

  // // TODO: ???
  // WiFi.begin(ssid, password);

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

#ifdef VELOCITY

  currentMillis = millis();

  // PID Velocity
  // if (currentMillis - previousMillisPIDVelocity > interval_pid_velocity) {
  //   for (int i = 0; i < WHEEL_COUNT; i++) {
  //     wheels[i].tuningRPM();
  //   }
  //   previousMillisPIDVelocity = currentMillis;
  // }

  

  // Velocity
  if (currentMillis - previousMillis > interval_pid_velocity) {

      // Info velocity
    if (currentMillis - previousMillisInfoVelocity > interval_velocity_info) {
      // for (int i = 0; i < WHEEL_COUNT; i++) {
      //   wheels[i].infoVelocity();
      // }
      mecanumCar.carInfo();
      previousMillisInfoVelocity = currentMillis;
    }

    for(int i = 0; i < WHEEL_COUNT; i++){
      wheels[i].tuningRPM();
      
      if (wheels[i].getDirection() == RIGHT_DIR) {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      } else {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      }
      wheels[i].resetEncValue(); // Reset encoder value
    }
    mecanumCar.updateVelocity();
    mecanumCar.updatePosition();
    // mecanumCar.carInfo();
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
      // wheels[i].infoPosition();
      wheels[i].tuningPosition();
      if (wheels[i].getDirection() == LEFT_DIR) {
        wheels[i].getMotor().TurnLeft(wheels[i].getPWM());
      } else {
        wheels[i].getMotor().TurnRight(wheels[i].getPWM());
      }
    }
    previousMillis = currentMillis;
  }

  if (currentMillis - previousMillisInfoPosition > interval_pid_position)
  {
    for (int i = 0; i < WHEEL_COUNT; i++)
    {
      wheels[i].infoPosition();
    }
    previousMillisInfoPosition = currentMillis;
  }

  if (currentMillis - previousMillisPIDPosition > interval_pid_position)
  {

      for (int i = 0; i < WHEEL_COUNT; i++)
      {
        // positions[cur] += 130;
        wheels[i].setTargetPosition(positions[i]);
      }

    previousMillisPIDPosition = currentMillis;
  }

  SerialDataWrite();

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
    const operation_mode_t op_mode = doc["m"];
    switch (op_mode) {
      case (JOYSTICK_MANUAL): {
        const double vx = doc["x"], vy = doc["y"];
        const uint8_t op_type = doc["t"];
        
        switch (op_type) {
          case (OMNIDIRECTIONAL): {
            // const uint8_t throttle = doc["th"];
            mecanumCar.move(vx*velo_test, vy*velo_test, 0);
            break;
          }
          case (ROTATIONAL): {
            // const double wz = doc["z"];
            // mecanumCar.move(0, 0, wz*0.2);
            break;
          }
          default: {
            Serial.println("unknown movement type");
          }
        }

        break;
      }

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
        mecanumCar.move(vx * velo_test, vy * velo_test, 0);

        break;
      }
      
      case (BUTTONS_AUTO): {
        // {"m":2,"step":1,"p1":"12.887","p2":"12.887","p3":"12.887","p4":"12.887"}
        const double p1 = doc["p1"], p2 = doc["p2"], p3 = doc["p3"], p4 = doc["p4"];
        const uint8_t step = doc["step"];
        // const double vx = doc["p1"], vy = doc["p2"];
        // const uint8_t throttle = doc["th"];

        Serial.println(step);
        Serial.println(p1);
        Serial.println(p2);
        Serial.println(p3);
        Serial.println(p4);
        positions[0] += p1;
        // positions = curpos + p1
        positions[1] += p2;
        positions[2] += p3;
        positions[3] += p4;

        // Serial.print(vx);
        // Serial.print(" ");
        // Serial.print(vy);
        // Serial.print(" ");
        // Serial.print(throttle);
        // Serial.println();

        // move(vx*0.2 , vy*0.2, 0);
        // mecanumCar.move(vx*0.2, vy*0.2, 0);

        break;
      } 

      default: {
        Serial.println("unknown operaiton mode");
        break;
      }
    }
    // const movement_t mv_type = doc["t"];
    // switch (mv_type) {
    //   case (OMNIDIRECTIONAL): {
    //     const operation_mode_t op_type = doc["m"];

    //     switch (op_type) {
    //       case (BUTTONS_MANUAL): {
    //         const double vx = doc["x"], vy = doc["y"];
    //         const uint8_t throttle = doc["th"];

    //         Serial.print(vx);
    //         Serial.print(" ");
    //         Serial.print(vy);
    //         Serial.print(" ");
    //         Serial.print(throttle);
    //         Serial.println();

    //         // move(vx*0.2 , vy*0.2, 0);
    //         mecanumCar.move(vx*0.2, vy*0.2, 0);

    //         break;
    //       }

    //       default: {
    //         Serial.println("unknown operaiton mode");
    //       }
    //     }

    //     break;
    //   }
    //   case (ROTATIONAL): {
        
    //     break;
    //   }
    //   default: {
    //     Serial.println("unknown movement type");
    //   }
    // }
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
      int option = received_chars.toInt();
      char c = received_chars[0];
      command_test(c);
      received_chars = "";
    }
  }
}

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

