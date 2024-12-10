#include "Motor.h"
#include "Sever.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// Encoder output to Arduino Interrupt pin
#define ENC_IN_1 13
#define ENC_IN_2 35
#define ENC_IN_B 26
#define ENC_IN_A 13

volatile int posi = 0; // Position updated by the encoder
volatile int previousPos = 0; // Previous position


// Pin configurations for motor 1 and motor 2
motorPin motorPin1 = motorPin(13, 26, 27, 14, 12); // pinA, pinB, EN, L_PWM, R_PWM
// motorPin motor2Pin = motorPin(25, 33, 32); // EN, L_PWM, R_PWM

// Motor motorControllers[] = {
//   Motor(motor1Pin.EN, motor1Pin.L_PWM, motor1Pin.R_PWM),
//   Motor(motor2Pin.EN, motor2Pin.L_PWM, motor2Pin.R_PWM)
// };

const int NUM_MOTORS = 1;

volatile long encoderValues[] = {0, 0};
std::vector<int> rpms = {0, 0};
// std::vector<int> motorPwms = {50, 100};
std::vector<int> encoderPinsA = {13, 35};
std::vector<int> encoderPinsB = {26, 25};

// PID control variables
double targetRPMs[] = {30.0, 100.0};  // Set your target RPM values
double currentRPMs[] = {0.0, 0.0};     // Current RPM of each motor
double computedPWMs[] = {0.0, 0.0};       // PWM output to motors

// tune PID
double previousPWMs[] = {0.0, 0.0}; 

// PID constants (to be tuned)
double Kp = 0.8, Ki = 0.0, Kd = 0.0;

// std::vector<PID> motorPIDs = {
//   PID(&currentRPMs[0], &computedPWMs[0], &targetRPMs[0], Kp, Ki, Kd, DIRECT),
//   PID(&currentRPMs[1], &computedPWMs[1], &targetRPMs[1], Kp, Ki, Kd, DIRECT)
// };

// PID constants
double kp = 1.0;
double kd = 0;
double ki = 0.0;

// Declare PID variables
double input = 0;      // Current position (input for PID)
double output = 0;     // Output from PID (motor speed)
double setpoint = 0;   // Target position for the motor

// Create a PID instance
// PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Define a type for function pointers (callback functions)
typedef void (*CallbackFunction)();

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

long previousEncoderValues = 0;

Wheel wheelsSpeed[] = {
  Wheel(motorPin1.pinA, motorPin1.pinB, motorPin1.EN, motorPin1.L_PWM, motorPin1.R_PWM, 0, 0, kp, kd, ki)
};

// Declare the trigger interrupt function
void triggerW1A() {wheelsSpeed[0].triggerA();};
void triggerW1B() {wheelsSpeed[0].triggerB();};

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  int pinA = wheelsSpeed[0].getEncoder().getPinA();
  attachInterrupt(digitalPinToInterrupt(pinA), triggerW1A, RISING);
  wheelsSpeed[0].getMotor().Enable();
  wheelsSpeed[0].getMotor().TurnRight(50);
  wheelsSpeed[0].setSetpoint(150); 

  // initWiFi();
  // startServer();
  
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    int pinA = wheelsSpeed[0].getEncoder().getPinA();
    double currentRPM = (float)wheelsSpeed[0].getEncoderValue() * 60 / ENC_COUNT_REV;
    wheelsSpeed[0].tuningRPM(currentRPM);
    wheelsSpeed[0].info();
    wheelsSpeed[0].getMotor().TurnRight(wheelsSpeed[0].getPWM());
    previousMillis = currentMillis;
  }
}
