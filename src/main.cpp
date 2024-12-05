#include "BTS7960.h"
#include "temp.h"
#include "util.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// Encoder output to Arduino Interrupt pin
#define ENC_IN_1 13
#define ENC_IN_2 35

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

// Pin configurations for motor 1 and motor 2
motorPin motor1Pin = motorPin(27, 14, 12); // EN, L_PWM, R_PWM
motorPin motor2Pin = motorPin(25, 33, 32); // EN, L_PWM, R_PWM

BTS7960 motorControllers[] = {
  BTS7960(motor1Pin.EN, motor1Pin.L_PWM, motor1Pin.R_PWM),
  BTS7960(motor2Pin.EN, motor2Pin.L_PWM, motor2Pin.R_PWM)
};

const int NUM_MOTORS = 1;

volatile long encoderValues[] = {0, 0};
std::vector<int> rpms = {0, 0};
// std::vector<int> motorPwms = {50, 100};
std::vector<int> encoderPins = {13, 35};

// PID control variables
double targetRPMs[] = {150.0, 100.0};  // Set your target RPM values
double currentRPMs[] = {0.0, 0.0};     // Current RPM of each motor
double computedPWMs[] = {0.0, 0.0};       // PWM output to motors

// tune PID
double previousPWMs[] = {0.0, 0.0}; 

// PID constants (to be tuned)
double Kp = 0.8, Ki = 0.0, Kd = 0.0;

std::vector<PID> motorPIDs = {
  PID(&currentRPMs[0], &computedPWMs[0], &targetRPMs[0], Kp, Ki, Kd, DIRECT),
  PID(&currentRPMs[1], &computedPWMs[1], &targetRPMs[1], Kp, Ki, Kd, DIRECT)
};

// Define a type for function pointers (callback functions)
typedef void (*CallbackFunction)();

// Declare the updateEncoder function
void updateEncoder1();
void updateEncoder2();
std::vector<CallbackFunction> updateEncoders = {updateEncoder1, updateEncoder2};

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  for (int i = 0; i < NUM_MOTORS; i++){
    pinMode(encoderPins[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPins[i]), updateEncoders[i], RISING);
  }

  for(int i = 0; i < NUM_MOTORS; i++) {
    motorControllers[i].Enable();
    motorControllers[i].TurnRight(currentRPMs[i]);
    
    // Initialize PID controller for each motor
    motorPIDs[i].SetMode(AUTOMATIC);  // Enable PID control
    motorPIDs[i].SetOutputLimits(-255, 255); // Output PWM limits (-255->255)
    motorPIDs[i].SetSampleTime(interval); // Set PID sample time to interval
  }

  // initWiFi();
  // startServer();
}

void loop() {
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    int duration = currentMillis - previousMillis;
    previousMillis = currentMillis;

    Serial.println("=========================================");

    for(int i = 0; i < NUM_MOTORS; i++) {
      // Calculate current RPM
      currentRPMs[i] = (float)(encoderValues[i] * 60 / ENC_COUNT_REV);
      // currentRPMs[i] = rpms[i]; // Update current RPM for PID computation

      // Compute PID output
      motorPIDs[i].Compute();
      previousPWMs[i] += computedPWMs[i]; 
      // Apply the PID output to the motor
      double error = targetRPMs[i] - currentRPMs[i];

      // Print status
      Serial.print("MOTOR ");
      Serial.println(i + 1);
      Serial.print("DURATION: ");
      Serial.print(duration);
      Serial.print('\t');
      Serial.print("PWM VALUE: ");
      Serial.print(previousPWMs[i]);
      Serial.print('\t');
      Serial.print("PULSES/Encoder Values: ");
      Serial.print(encoderValues[i]);
      Serial.print('\t');
      Serial.print("SPEED: ");
      Serial.print(currentRPMs[i]);
      Serial.println(" RPM");

      // (Optional) Print PID terms if supported by your library
      Serial.print("P Term: ");
      Serial.print(motorPIDs[i].GetKp() * error);
      Serial.print(" I Term: ");
      Serial.print(motorPIDs[i].GetKi());
      // Add logic to calculate/display the integral term if needed
      Serial.print(" D Term: ");
      Serial.print(motorPIDs[i].GetKd());
      Serial.print(" Error: ");
      Serial.println(error);
      // Add logic to calculate/display the derivative term if needed

      encoderValues[i] = 0;
      // motorControllers[i].TurnRight((int)computedPWMs[i]);
      motorControllers[i].TurnRight(previousPWMs[i]);
    }
  }
}

void updateEncoder1() {
  encoderValues[0]++;
}

void updateEncoder2() {
  encoderValues[1]++;
}
