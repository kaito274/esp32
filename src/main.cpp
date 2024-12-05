#include "BTS7960.h"
#include "temp.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// Encoder output to Arduino Interrupt pin
#define ENC_IN_1 13
#define ENC_IN_2 35

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

// Pin configurations for motor 1 and motor 2
const int motor1Pins[] = {27, 14, 12};  // EN, L_PWM, R_PWM for motor 1
const int motor2Pins[] = {25, 33, 32};  // EN, L_PWM, R_PWM for motor 2

BTS7960 motorControllers[] = {
  BTS7960(motor1Pins[0], motor1Pins[1], motor1Pins[2]),
  BTS7960(motor2Pins[0], motor2Pins[1], motor2Pins[2])
};

const int NUM_MOTORS = 1;

volatile long encoderValues[] = {0, 0};
std::vector<int> rpms = {0, 0};
// std::vector<int> motorPwms = {50, 100};
std::vector<int> encoderPins = {13, 35};

// PID control variables
double targetRPMs[] = {150.0, 100.0};  // Set your target RPM values
// double targetPWMs[] = {
//   map(targetRPMs[0], 0, 333, 0, 255),
//   map(targetRPMs[1], 0, 33, 0, 255)
// };
double currentRPMs[] = {0.0, 0.0};     // Current RPM of each motor
double computedPWMs[] = {0.0, 0.0};       // PWM output to motors
// double motorRPMS[] = {0.0, 0.0};       // RPM output to motors

double currentPWMs[] = {0.0, 0.0};     // Current PWM of each motor

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

// tune PID
double previousPWMS = 0;
double deltaPWMS = 5;

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  for (int i = 0; i < NUM_MOTORS; i++){
    pinMode(encoderPins[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPins[i]), updateEncoders[i], RISING);
  }

  for(int i = 0; i < NUM_MOTORS; i++) {
    motorControllers[i].Enable();
    // motorControllers[i].Stop();
    motorControllers[i].TurnRight(currentRPMs[i]);
    
    // Initialize PID controller for each motor
    motorPIDs[i].SetMode(AUTOMATIC);  // Enable PID control
    motorPIDs[i].SetOutputLimits(-255, 255); // Output PWM limits (0-255)
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
      rpms[i] = (float)(encoderValues[i] * 60 / ENC_COUNT_REV);
      currentRPMs[i] = rpms[i]; // Update current RPM for PID computation

      // Compute PID output
      motorPIDs[i].Compute();

      // Apply the PID output to the motor
      double error = targetRPMs[i] - currentRPMs[i];
      // if(abs(error) < 2){
      //   Serial.println("Maintaining constant speed... ");
      // }else if(error > 0){
      //   motorControllers[i].TurnRight((int)computedPWMs[i]);
      //   Serial.print("Increasing speed... ");
      //   Serial.println(computedPWMs[i]);
      // }else{ //error < 0
       
      // }

      // if (abs(error) < 2) {
      //   // Error is within acceptable range, maintain constant speed
      //   motorControllers[i].TurnRight((int)motorPWMs[i]);
      //   Serial.print("Maintaining constant speed... ");
      //   Serial.println(motorPWMs[i]);
      // } else if (error > 0) {
      //   // Positive error, increase speed to reach target
      //   motorControllers[i].TurnRight((int)motorPWMs[i]);
      //   Serial.print("Increasing speed... ");
      //   Serial.println(motorPWMs[i]);
      // } else {
      //   // Negative error, slow down to match target
      //   double reducedPWM = motorPWMs[i] * 0.9; // Reduce PWM by 10% (example adjustment)
      //   motorControllers[i].TurnRight((int)reducedPWM);
      //   Serial.print("Slowing down... ");
      //   Serial.println(reducedPWM);
      // }

      // Print status
      Serial.print("MOTOR ");
      Serial.println(i + 1);
      Serial.print("DURATION: ");
      Serial.print(duration);
      Serial.print('\t');
      Serial.print("PWM VALUE: ");
      Serial.print(computedPWMs[i]);
      Serial.print('\t');
      Serial.print("PULSES/Encoder Values: ");
      Serial.print(encoderValues[i]);
      Serial.print('\t');
      Serial.print("SPEED: ");
      Serial.print(rpms[i]);
      Serial.println(" RPM");


      // (Optional) Print PID terms if supported by your library
      Serial.print("P Term: ");
      Serial.print(motorPIDs[i].GetKp());
      Serial.print("I Term: ");
      Serial.print(motorPIDs[i].GetKi());
      // Add logic to calculate/display the integral term if needed
      Serial.print("D Term: ");
      Serial.print(motorPIDs[i].GetKd());
      Serial.print(" Error: ");
      Serial.println(error);
      // Add logic to calculate/display the derivative term if needed

      // Reset encoder count for the next interval
      // motorControllers[i].TurnRight(currentRPMs[i]); // Manually send 0 PWM
      encoderValues[i] = 0;
      previousPWMS += computedPWMs[i];
      // motorControllers[i].TurnRight((int)computedPWMs[i]);
      motorControllers[i].TurnRight(previousPWMS);
    }
  }
}

void updateEncoder1() {
  encoderValues[0]++;
}

void updateEncoder2() {
  encoderValues[1]++;
}
