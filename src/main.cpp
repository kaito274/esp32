#include "Motor.h"
// #include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
// #include "JSON_UARTReader.h"
// #include "config.h"
#include <WiFi.h>
#include <PID_v1.h>
#include <ArduinoJson.h>

#define LED 2

// Server settings
// #ifndef CONFIG_H
// #define CONFIG_H

const char *ssid_ = "@@@@";
const char *password_ = "khongcopass";

// #endif// Replace with your network credentials


#define UART_BUFFER_SIZE 256
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
  WiFi.begin(ssid_, password_);

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

void sendDataToClient(void *parameter);

// // PID variables
// double setpoint;    // Desired motor velocity (RPM)
// double input;       // Current motor velocity (RPM)
// double output;      // PID output (PWM value)

// // PID tuning parameters
// double Kp = 2.0, Ki = 5.0, Kd = 1.0;

// // Create PID instance
// PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// // Variables for encoder
// volatile long encoderCount = 0;  // Encoder pulse count
// unsigned long prevTime = 0;      // Previous time for velocity calculation
// double rpm = 0;                  // Motor velocity in RPM

void move(double vx, double vy, double wz);


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
  // xTaskCreatePinnedToCore(
  //   sendDataTask,    // Task function
  //   "SendDataTask",  // Name of the task
  //   10000,           // Stack size (in words)
  //   NULL,            // Task parameter
  //   5,               // Priority (higher value = higher priority)
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


  // Initialize PID
  // myPID.SetMode(AUTOMATIC);
  // myPID.SetOutputLimits(0, 255);  // PWM range
  // setpoint = 100;  // Set desired RPM

}

void SerialDataWrite();
void command_test(char option);

void loop()
{
  // WiFiClient client = server.available();

#ifdef VELOCITY
  // Velocity
  currentMillis = millis();

  if (currentMillis - previousMillis > interval_velocity)
  {
  
    // wheels[0].tuningRPM();
    // wheels[1].tuningRPM();
    // wheels[2].tuningRPM();
    // wheels[3].tuningRPM();
      // analogWrite(26, 100);
      // analogWrite(27, 100);
    if (wheels[0].getDirection() == RIGHT_DIR) {
      wheels[0].getMotor().TurnRight(wheels[0].getPWM());
    } else {
      wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
    }
 
    if (wheels[1].getDirection() == RIGHT_DIR) {
      wheels[1].getMotor().TurnRight(wheels[1].getPWM());
    } else {
      wheels[1].getMotor().TurnLeft(wheels[1].getPWM());
    }
    
    if (wheels[2].getDirection() == RIGHT_DIR) {
      wheels[2].getMotor().TurnRight(wheels[2].getPWM());
    } else {
      wheels[2].getMotor().TurnLeft(wheels[2].getPWM());
    }

    if (wheels[3].getDirection() == RIGHT_DIR) {
      wheels[3].getMotor().TurnRight(wheels[3].getPWM());
    } else {
      wheels[3].getMotor().TurnLeft(wheels[3].getPWM());
    }
    previousMillis = currentMillis;

    // TODO: test Phuc
    // Serial.print(wheels[3].getTargetRPM());
    // Serial.print(" ");
    // Serial.println(wheels[3].getComputedPWMVelocity());
    // Serial.print(wheels[3].getPIDVelocity().GetKp());
    // Serial.print(" ");
    // Serial.print(wheels[3].getPIDVelocity().GetKi());
    // Serial.print(" ");
    // Serial.println(wheels[3].getPIDVelocity().GetKd());
    wheels[0].infoVelocity();
    wheels[1].infoVelocity();
    wheels[2].infoVelocity();
    wheels[3].infoVelocity();

    // Reset encoder value
    wheels[0].resetEncValue();
    wheels[1].resetEncValue();
    wheels[2].resetEncValue();
    wheels[3].resetEncValue();
  }
  
  SerialDataWrite();
#endif // VELOCITY

#ifdef POSITION
  // Position
  currentMillis = millis();
  if (currentMillis - previousMillis > interval_position)
  {
    wheels[0].infoPosition();
    wheels[0].tuningPosition();
    if (wheels[0].getDirection() == LEFT_DIR) {
      wheels[0].getMotor().TurnLeft(wheels[0].getPWM());
    } else {
      wheels[0].getMotor().TurnRight(wheels[0].getPWM());
    }

    // wheels[1].infoPosition();
    // wheels[1].tuningPosition();
    // if (wheels[1].getDirection() == LEFT_DIR) {
    //   wheels[1].getMotor().TurnLeft(wheels[1].getPWM());
    // } else {
    //   wheels[1].getMotor().TurnRight(wheels[1].getPWM());
    // }

    // wheels[2].infoPosition();
    // wheels[2].tuningPosition();
    // if (wheels[2].getDirection() == LEFT_DIR) {
    //   wheels[2].getMotor().TurnLeft(wheels[2].getPWM());
    // } else {
    //   wheels[2].getMotor().TurnRight(wheels[2].getPWM());
    // }

    // wheels[3].infoPosition();
    // wheels[3].tuningPosition();
    // if (wheels[3].getDirection() == LEFT_DIR) {
    //   wheels[3].getMotor().TurnLeft(wheels[3].getPWM());
    // } else {
    //   wheels[3].getMotor().TurnRight(wheels[3].getPWM());
    // }
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

            move(vx* 0.2 , vy*0.2, 0);

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


  //  // Calculate RPM every 100 ms
  // unsigned long currentTime = millis();
  // if (currentTime - prevTime >= 100) {
  //   // noInterrupts();
  //   // long count = encoderCount;  // Copy encoder count
  //   // encoderCount = 0;           // Reset encoder count
  //   // interrupts();

  //   // // Calculate RPM
  //   // rpm = (count / 20.0) * (600.0 / 0.1);  // Assuming 20 pulses per revolution
  //   // input = rpm;

  //   // Compute PID output
  //   myPID.Compute();

  //   // // Apply motor control
  //   // motorControl(output);

  //   // Debugging
  //   Serial.print("Setpoint: ");
  //   Serial.print(setpoint);
  //   Serial.print(" RPM, Input: ");
  //   Serial.print(input);
  //   Serial.print(" RPM, Output: ");
  //   Serial.println(output);

  //   prevTime = currentTime;
  // }
  // SerialDataWrite();

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
    move(-velo_test, velo_test, 0.0);
    break;
  case '2':
    move(-velo_test, 0, 0.0);
    break;
  case '3':
    move(-velo_test, -velo_test, 0.0);
    break;
  case '4':
    move(0, velo_test, 0.0);
    break;
  case '5':
    move(0.0, 0.0, 0.0);
    Serial.println("Stopped");
    break;
  case '6':
    move(0, -velo_test, 0.0);
    break;
  case '7':
    move(velo_test, velo_test, 0.0);
    break;
  case '8':
    move(velo_test, 0, 0.0);
    break;
  case '9':
    move(velo_test, -velo_test, 0.0);
    break;
  case 'p': //rotate right
    move(0, 0, -velo_rotate);
    break;
  case 'o': //rotate left
    move(0, 0, velo_rotate);
    break;
  case 'k': //drift left
    move(0.0, velo_test, velo_rotate);
    break;
  case 'l': //drift rightp
  
    move(0.0, -velo_test, -velo_rotate);
    break;
  default:
    move(0, 0, 0);
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
  // rpmFL = constrain(rpmFL, -MAX_RPM, MAX_RPM);
  // rpmFR = constrain(rpmFR, -MAX_RPM, MAX_RPM);
  // rpmRL = constrain(rpmRL, -MAX_RPM, MAX_RPM);
  // rpmRR = constrain(rpmRR, -MAX_RPM, MAX_RPM);

  // Set direction of the wheels
  int dirFL = (w_fl > 0) ? 1 : -1;
  int dirFR = (w_fr > 0) ? 1 : -1;
  int dirRL = (w_rl > 0) ? 1 : -1;
  int dirRR = (w_rr > 0) ? 1 : -1;

  // // Convert RPMs to PWM values (absolute values)
  pwmFL = map(abs(rpmFL), 0, MAX_RPM, 0, MAX_PWM);
  pwmFR = map(abs(rpmFR), 0, MAX_RPM, 0, MAX_PWM);
  pwmRL = map(abs(rpmRL), 0, MAX_RPM, 0, MAX_PWM);
  pwmRR = map(abs(rpmRR), 0, MAX_RPM, 0, MAX_PWM);

  // // Apply the computed PWM values to the motors
  wheels[0].setPWM(pwmFL);
  wheels[1].setPWM(pwmFR);
  wheels[2].setPWM(pwmRL);
  wheels[3].setPWM(pwmRR);

  // TODO: Using PIDvelo_rotate
  // wheels[0].setTargetRPM(abs(rpmFL));
  // wheels[1].setTargetRPM(abs(rpmFR));
  // wheels[2].setTargetRPM(abs(rpmRL));
  // wheels[3].setTargetRPM(abs(rpmRR));

  // Set the direction of the wheels
  wheels[0].setDirection(dirFL);
  wheels[1].setDirection(dirFR);
  wheels[2].setDirection(dirRL);
  wheels[3].setDirection(dirRR);
}