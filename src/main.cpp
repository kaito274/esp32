#include "Motor.h"
#include "Server.h"
#include "util.h"
#include "Encoder.h"
#include "Wheel.h"
#include "GlobalSettings.h"
#include <PID_v1.h>
#include <vector>
#define LED 2

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// volatile long posi = 0; // Position updated by the encoder
// volatile long encoderValues = 0; // Encoder value updated by the encoder

// // PID constants
// double kp_ = 1.0;
// double kd_ = 0.0;
// double ki_ = 0.0;
// // Declare PID variables
// double input = 0;      // Current position (input for PID)
// double output = 0;     // Output from PID (motor speed)
// double setpoint = 0;   // Target position for the motor
// // Define a type for function pointers (callback functions)
// typedef void (*CallbackFunction)();
// void updateEncoderB();
// void updateEncoderA();

// #define ENC_IN_A 4
// #define ENC_IN_B 5

// // Create a PID instance
// PID myPID(&input, &output, &setpoint, kp_, ki_, kd_, DIRECT);
// Motor motor1(15, 2); // EN, L_PWM, R_PWM

void setup()
{
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  // Attach the interrupt functions to the encoder pins
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[0].getEncPinA()), triggerW1A, RISING);
  attachInterrupt(digitalPinToInterrupt(wheelsSpeed[0].getEncPinA()), triggerW1B, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[1].getEncPinA()), triggerW2A, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[1].getEncPinA()), triggerW2B, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[2].getEncPinA()), triggerW3A, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[2].getEncPinA()), triggerW3B, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[3].getEncPinA()), triggerW4A, RISING);
  // attachInterrupt(digitalPinToInterrupt(wheelsSpeed[3].getEncPinA()), triggerW4B, RISING);

  wheelsSpeed[0].getMotor().Enable();
  // wheelsSpeed[0].getMotor().TurnRight(50);

  // // initWiFi();
  // // startServer();

  

  // pinMode(ENC_IN_A, INPUT_PULLUP);
  // pinMode(ENC_IN_B, INPUT);
  
  
  // attachInterrupt(digitalPinToInterrupt(ENC_IN_A), updateEncoderA, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENC_IN_A), updateEncoderB, RISING);
  // motor1.Enable();
  // setpoint = 333;  // Set the target position for the motor
  // myPID.SetMode(AUTOMATIC);  // Enable PID control
  // myPID.SetOutputLimits(-255, 255); // Output PWM limits (-255->255)
  // motor1.TurnRight(50);

  // TODO: TEST
  // wheelsSpeed[0].setTargetRPM(100);

  wheelsSpeed[0].setTargetPosition(333);
}

void loop()
{
  currentMillis = millis();
  // if (currentMillis - previousMillis > interval)
  // {

  //   wheelsSpeed[0].tuningRPM();
  //   wheelsSpeed[0].info();
  //   if (dir == 1)
  //   {
  //     wheelsSpeed[0].getMotor().TurnRight(wheelsSpeed[0].getPWM());
  //   }
  //   else
  //   {
  //     wheelsSpeed[0].getMotor().TurnLeft(wheelsSpeed[0].getPWM());
  //   }

  //   previousMillis = currentMillis;
  // }


  // noInterrupts();
  wheelsSpeed[0].setCurrentPosition(wheelsSpeed[0].getEncPosition());
  // Serial.println(wheelsSpeed[0].getEncPosition());
  // interrupts();

  // Compute the PID output
  wheelsSpeed[0].getPIDPosition().Compute();

  double error = wheelsSpeed[0].getTargetPosition() - wheelsSpeed[0].getCurrentPosition();

  // Apply the PID output to the motor (convert to PWM)
  int pwr = (int)fabs(error);  // Convert output to absolute power value
  pwr = constrain(pwr, 0, 100); // Ensure the PWM value is within valid range (0 to 255)

  // Determine the direction (sign of the output)
  int dir = (error < 0) ? -1 : 1;
  
  if(dir > 0 ){
    wheelsSpeed[0].getMotor().TurnLeft(pwr);
  } else {
    wheelsSpeed[0].getMotor().TurnRight(pwr);
  }

  Serial.print("Dir: "); Serial.print(dir);
  Serial.print("\tTarget: "); Serial.print(wheelsSpeed[0].getTargetPosition());
  Serial.print("\tCurrent position: "); Serial.print(wheelsSpeed[0].getEncPosition());
  Serial.print("\tMotor Power: "); Serial.print(pwr);
  Serial.print("\tError: "); Serial.println(error);
  
  
  // // Print debug information
  // if (currentMillis - previousMillis > interval)
  // {
  //   Serial.print("Encoder A: "); Serial.println(wheelsSpeed[0].getEncPinA());
  //   Serial.print("Encoder B: "); Serial.println(wheelsSpeed[0].getEncPinB());
  //   Serial.print("Dir: "); Serial.print(dir);
  //   Serial.print("\tTarget: "); Serial.print(wheelsSpeed[0].getTargetPosition());
  //   Serial.print("\tCurrent position: "); Serial.print(wheelsSpeed[0].getEncPosition());
  //   Serial.print("\tMotor Power: "); Serial.print(pwr);
  //   Serial.print("\tError: "); Serial.println(error);
  //   previousMillis = currentMillis;
  // }


  // motor1.TurnRight(50);

  // // noInterrupts();
  // input = posi;
  // // interrupts();

  // // Compute the PID output
  // myPID.Compute();

  // double error = setpoint - input;

  // // Apply the PID output to the motor (convert to PWM)
  // int pwr = (int)fabs(error);  // Convert output to absolute power value
  // pwr = constrain(pwr, 0, 100); // Ensure the PWM value is within valid range (0 to 255)

  // // Determine the direction (sign of the output)
  // int dir = (error < 0) ? -1 : 1;
  
  // if(dir > 0 ){
  //   motor1.TurnLeft(pwr);
  // } else {
  //   motor1.TurnRight(pwr);
  // }
  
  // Serial.print("Target: "); Serial.print(setpoint);
  // Serial.print("\tCurrent position: "); Serial.print(input);
  // Serial.print("\tMotor Power: "); Serial.print(pwr);
  // Serial.print("\tError: "); Serial.println(error);
}

// void updateEncoderA() {
//   encoderValues++;
// }

// void updateEncoderB() {
//   int B = digitalRead(ENC_IN_B);
//   // Serial.println(B);
//     if (B > 0) {
//         posi++;
//     } else {
//         posi--;
//     }
// }
