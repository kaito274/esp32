/***************************************************
Copyright (c) 2019 Luis Llamas
(www.luisllamas.es)
Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License
 ****************************************************/
 
#ifndef __MOTOR_H__
#define __MOTOR_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Motor
{
  public:
  Motor(uint8_t L_PWM, uint8_t R_PWM) : Motor(0, 0, L_PWM, R_PWM) {}
	Motor(uint8_t EN, uint8_t L_PWM, uint8_t R_PWM) : Motor(EN, 0, L_PWM, R_PWM) {}
  Motor(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
  void Enable();
  void Disable();

	void TurnLeft(uint8_t pwm);
	void TurnRight(uint8_t pwm);
  void Brake(uint8_t pwm);
	void Stop();

  private:
    uint8_t _L_EN;
    uint8_t _R_EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;
};

#endif