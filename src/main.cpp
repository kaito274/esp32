/***************************************************
Copyright (c) 2019 Luis Llamas
(www.luisllamas.es)
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/

// PINOUT
// L_EN -> 8
// R_EN -> 8
// L_PWM -> 9
// R_PWM -> 10

#include "BTS7960.h"
#include "temp.h"

#define LED 2

// Encoder output to Arduino Interrupt pin
#define ENC_IN_1 13
#define ENC_IN_2 35

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

const uint8_t EN_1 = 27;
const uint8_t L_PWM_1 = 14;
const uint8_t R_PWM_1 = 12;

const uint8_t EN_2 = 25;
const uint8_t L_PWM_2 = 33;
const uint8_t R_PWM_2 = 32;

BTS7960 motorController1(EN_1, L_PWM_1, R_PWM_1);
BTS7960 motorController2(EN_2, L_PWM_2, R_PWM_2);

// Analog pin for potentiometer
int speedcontrol = 0;

// Pulse count from encoder
volatile long encoderValue_1 = 0;
volatile long encoderValue_2 = 0;

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

// Variable for RPM measuerment
int rpm1 = 0;
int rpm2 = 0;

// Variable for PWM motor speed output
int motorPwm1 = 50;
int motorPwm2 = 100;

// Declare the updateEncoder function
void updateEncoder1();
void updateEncoder2();

void setup()
{
  Serial.begin(115200);

  // pinMode(motor1Pin1, OUTPUT);
  // pinMode(motor1Pin2, OUTPUT);
  // pinMode(enable1Pin, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(ENC_IN_1, INPUT_PULLUP);
  pinMode(ENC_IN_2, INPUT_PULLUP);
  motorController1.Enable();
  motorController2.Enable();
  motorController1.TurnRight(motorPwm1);
  motorController2.TurnRight(motorPwm2);

  // Attach interrupt to encoder
  attachInterrupt(digitalPinToInterrupt(ENC_IN_1), updateEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_2), updateEncoder2, RISING);

  // initWiFi();
  // startServer();
}

void loop()
{

  // motorController1.TurnRight(50);
  // motorController2.TurnLeft(100);
  // delay(5000);

  //  motorController.Enable();
  // for(int i = 1; i < 255; i++){
  //   motorController.TurnRight(i);
  //   delay(50);
  // }
  // motorController.Stop();
  // motorController.Disable();

  // motorController.TurnRight(motorPwm);

  currentMillis = millis();
  // Serial.println(currentMillis);
  // Update RPM value every second
  // Serial.println(currentMillis);
  if (currentMillis - previousMillis > interval)
  {

    int duration = currentMillis - previousMillis;
    previousMillis = currentMillis;

    // Calculate RPM
    rpm1 = (float)(encoderValue_1 * 60 / ENC_COUNT_REV);
    rpm2 = (float)(encoderValue_2 * 60 / ENC_COUNT_REV);
    // Only update display when there is a reading
    if (motorPwm1 > 0 || rpm2 > 0)
    {
      Serial.println("MOTOR 1");
      Serial.print("DURATION: ");
      Serial.print(duration);
      Serial.print('\t');
      Serial.print("PWM VALUE: ");
      Serial.print(motorPwm1);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue_1);
      Serial.print('\t');
      Serial.print(" SPEED: ");
      Serial.print(rpm1);
      Serial.println(" RPM");
    }
    if (motorPwm2 > 0 || rpm2 > 0)
    {
      Serial.println("MOTOR 2");
      Serial.print("DURATION: ");
      Serial.print(duration);
      Serial.print('\t');
      Serial.print("PWM VALUE: ");
      Serial.print(motorPwm2);
      Serial.print('\t');
      Serial.print(" PULSES: ");
      Serial.print(encoderValue_2);
      Serial.print('\t');
      Serial.print(" SPEED: ");
      Serial.print(rpm2);
      Serial.println(" RPM");
    }
    Serial.println("================================================");

    encoderValue_1 = 0;
    encoderValue_2 = 0;
    // motorPwm = (motorPwm + 10) % 255;
    // motorPwm %= 255;
    // motorController1.TurnRight(motorPwm1);
    // motorController2.TurnRight(motorPwm1);
  }

  // Toggle led 2
  // digitalWrite(LED, HIGH);
  // Serial.println("Led is on");
  // delay(500);
  // digitalWrite(LED, LOW);
  // Serial.println("Led is off");
  // delay(500);
}

void updateEncoder1()
{
  // noInterrupts();
  encoderValue_1++;
  // interrupts();
}

void updateEncoder2()
{
  // noInterrupts();
  encoderValue_2++;
  // interrupts();
}