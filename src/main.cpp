/***************************************************
Copyright (c) 2019 Luis Llamas
(www.luisllamas.es)
Licensed under the Apache License, Version 2.0 (the "License"); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at http://www.apache.org/licenses/LICENSE-2.0 Unless required by
applicable law or agreed to in writing, software distributed under the License
is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
KIND, either express or implied. See the License for the specific language
governing permissions and limitations under the License
 ****************************************************/

#include "BTS7960.h"
#include "temp.h"
#include <vector>
#define LED 2

// Encoder output to Arduino Interrupt pin
#define ENC_IN_1 13
#define ENC_IN_2 35

// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 330

// Pin configurations for motor 1 and motor 2
const int motor1Pins[] = {27, 14, 12};  // EN, L_PWM, R_PWM for motor 1
const int motor2Pins[] = {25, 33, 12};  // EN, L_PWM, R_PWM for motor 2

BTS7960 motorControllers[] = {
  BTS7960(motor1Pins[0], motor1Pins[1], motor1Pins[2]),
  BTS7960(motor2Pins[0], motor2Pins[1], motor2Pins[2])
};

const int NUM_MOTORS = 2;

volatile long encoderValues[NUM_MOTORS] = {0, 0};
std::vector<int> rpms = {0, 0};
std::vector<int> motorPwms = {50, 100};
std::vector<int> encoderPins = {13, 35};
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

  for(auto motorController : motorControllers) {
    motorController.Enable();
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
    // Only update display when there is a reading
    for(int i = 0; i < NUM_MOTORS; i++) {
      rpms[i] = (float)(encoderValues[i] * 60 / ENC_COUNT_REV);
      if (motorPwms[i] > 0 || rpms[i] > 0 ){
        Serial.print("MOTOR ");
        Serial.println(i + 1);
        Serial.print("DURATION: ");
        Serial.print(duration);
        Serial.print('\t');
        Serial.print("PWM VALUE: ");
        Serial.print(motorPwms[i]);
        Serial.print('\t');
        Serial.print(" PULSES: ");
        Serial.print(encoderValues[i]);
        Serial.print('\t');
        Serial.print(" SPEED: ");
        Serial.print(rpms[i]);
        Serial.println(" RPM");
      }
      encoderValues[i] = 0;
    }
    /*encoderValue_1 = 0;*/
    /*encoderValue_2 = 0;*/
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

void updateEncoder1() {
  encoderValues[0]++;
}

void updateEncoder2() {
  encoderValues[1]++;
}
