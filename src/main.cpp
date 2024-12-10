#include "BTS7960.h"
#include "temp.h"
#include "util.h"
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

std::vector<PID> motorPIDs = {
  PID(&currentRPMs[0], &computedPWMs[0], &targetRPMs[0], Kp, Ki, Kd, DIRECT),
  PID(&currentRPMs[1], &computedPWMs[1], &targetRPMs[1], Kp, Ki, Kd, DIRECT)
};

// PID constants
double kp = 1.0;
double kd = 0;
double ki = 0.0;

// Declare PID variables
double input = 0;      // Current position (input for PID)
double output = 0;     // Output from PID (motor speed)
double setpoint = 0;   // Target position for the motor

// Create a PID instance
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// Define a type for function pointers (callback functions)
typedef void (*CallbackFunction)();

// Declare the updateEncoder function
void updateEncoder1();
void updateEncoder2();
void updateEncoderB();
std::vector<CallbackFunction> updateEncoders = {updateEncoder1, updateEncoder2};

void setupTimerInterrupt();

// One-second interval for measurements
int interval = 1000;

// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;

long previousEncoderValues = 0;

// Function to make the motor turn exactly one full revolution to the right
void turnRightOneRound() {
  encoderValues[0] = 0;  // Reset encoder count to 0

  // Start the motor turning right
  motorControllers[0].TurnRight(50);  // Max PWM value, adjust as needed

  // Wait until the encoder counts one full revolution
  while (encoderValues[0] < ENC_COUNT_REV) {
    // Keep checking the encoder count while the motor is turning
    delay(10);  // Small delay to prevent overwhelming the processor with constant checks
  }

  // Stop the motor once one full revolution is completed
  // motorControllers[0].TurnRight(0);  // Stop the motor
  Serial.println("One full revolution completed!");
}

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  for (int i = 0; i < NUM_MOTORS; i++){
    pinMode(encoderPinsA[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPinsA[i]), updateEncoders[i], RISING);
  }
  // pinMode(ENC_IN_B, INPUT);
  // // pinMode(ENC_IN_A, INPUT);
  // attachInterrupt(digitalPinToInterrupt(ENC_IN_A), updateEncoderB, RISING);

  for(int i = 0; i < NUM_MOTORS; i++) {
    motorControllers[i].Enable();
    // motorControllers[i].TurnLeft(255);
    // Initialize PID controller for each motor
    motorPIDs[i].SetMode(AUTOMATIC);  // Enable PID control
    motorPIDs[i].SetOutputLimits(-255, 255); // Output PWM limits (-255->255)
    motorPIDs[i].SetSampleTime(interval); // Set PID sample time to interval
  }
  setpoint = 333 * 10;  // Set the target position for the motor
  myPID.SetMode(AUTOMATIC);  // Enable PID control
  myPID.SetOutputLimits(-255, 255); // Output PWM limits (-255->255)
  // myPID.SetSampleTime(interval); // Set PID sample time to interval
  // initWiFi();
  // startServer();
  setupTimerInterrupt();
  
}

hw_timer_t *timer = NULL;  // Hardware timer instance
volatile bool toggleLED = false;  // Flag to toggle the LED
volatile bool toggleMotor = false;  // Flag to toggle the motor
volatile int currentSpeed = 0;

int lastEncoderPulses = 0;

void loop() {
  // currentMillis = millis();
  // if (currentMillis - previousMillis > interval) {
  //   int duration = currentMillis - previousMillis;
  //   previousMillis = currentMillis;

  //   Serial.println("=========================================");

  //   for(int i = 0; i < NUM_MOTORS; i++) {
  //     // Calculate current RPM
  //     currentRPMs[i] = (float)(encoderValues[i] * 60 / ENC_COUNT_REV);
  //     // currentRPMs[i] = rpms[i]; // Update current RPM for PID computation

  //     // Compute PID output
  //     motorPIDs[i].Compute();
  //     previousPWMs[i] += computedPWMs[i]; 
  //     previousPWMs[i] = constrain(previousPWMs[i], 0, 255);
  //     // Apply the PID output to the motor
  //     double error = targetRPMs[i] - currentRPMs[i];

  //     // Print status
  //     Serial.print("MOTOR ");
  //     Serial.println(i + 1);
  //     Serial.print("DURATION: ");
  //     Serial.print(duration);
  //     Serial.print('\t');
  //     Serial.print("PWM VALUE: ");
  //     Serial.print(previousPWMs[i]);
  //     Serial.print('\t');
  //     Serial.print("PULSES/Encoder Values: ");
  //     Serial.print(encoderValues[i]);
  //     Serial.print('\t');
  //     Serial.print("SPEED: ");
  //     Serial.print(currentRPMs[i]);
  //     Serial.println(" RPM");

  //     // (Optional) Print PID terms if supported by your library
  //     Serial.print("P Term: ");
  //     Serial.print(motorPIDs[i].GetKp() * error);
  //     Serial.print(" I Term: ");
  //     Serial.print(motorPIDs[i].GetKi());
  //     // Add logic to calculate/display the integral term if needed
  //     Serial.print(" D Term: ");
  //     Serial.print(motorPIDs[i].GetKd());
  //     Serial.print(" Error: ");
  //     Serial.println(error);
  //     // Add logic to calculate/display the derivative term if needed

  //     encoderValues[i] = 0;
  //     // motorControllers[i].TurnRight((int)computedPWMs[i]);
  //     motorControllers[i].TurnRight(previousPWMs[i]);
  //   Serial.print("Current position: ");
  //   Serial.println(posi);
  //   // }
  // }
  // if(previousPos != posi){
  //   Serial.print("Current position: ");
  //   Serial.println(posi);
  //   previousPos = posi;
  // }
  // turnRightOneRound();  // Make the motor turn one full revolution to the right
  // delay(2000);

  // int duration = currentMillis - previousMillis;
  // previousMillis = currentMillis;
  // noInterrupts();
  // input = posi;
  // interrupts();

  // // Compute the PID output
  // myPID.Compute();

  // double error = setpoint - input;

  // // Apply the PID output to the motor (convert to PWM)
  // int pwr = (int)fabs(output);  // Convert output to absolute power value
  // pwr = constrain(pwr, 0, 100); // Ensure the PWM value is within valid range (0 to 255)

  // // Determine the direction (sign of the output)
  // int dir = (output < 0) ? -1 : 1;
  // // Serial.print("Dir: "); Serial.println(dir);
  // if(dir > 0 ){
  //   motorControllers[0].TurnLeft(pwr);
  // } else {
  //   motorControllers[0].TurnRight(pwr);
  // }

  // // Print debug information
  // // Serial.print("Duration: "); Serial.print(duration);
  // Serial.print("\tTarget: "); Serial.print(setpoint);
  // Serial.print("\tCurrent position: "); Serial.print(input);
  // Serial.print("\tMotor Power: "); Serial.print(pwr);
  // Serial.print("\tError: "); Serial.print(error);

  // MEASURE REAL RPM
  currentMillis = millis();
  // if (encoderValues[0] != previousEncoderValues) {
  //   noInterrupts();
  //   previousEncoderValues = encoderValues[0];
  //   Serial.println("=========================================");
  //   Serial.println("current encoder: " + String(encoderValues[0]));
  //   Serial.println("current time: " + String(currentMillis)); 
  //   interrupts();
  // }
  // if (currentMillis != previousMillis) {
  //   previousMillis = currentMillis;
  //   Serial.println("=========================================");
  //   Serial.println("currentMillis: " + String(currentMillis));
  //   Serial.println("current pulse: " + String(encoderValues[0]));
  // }
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;

  //   // Calculate RPM
  //   long pulseCount = encoderValues[0] - lastEncoderPulses;  // Get pulses in the last second
  //   long testrpm = (pulseCount * 60) / ENC_COUNT_REV;  // Calculate RPM

  //   // Print the RPM
  //   Serial.print("RPM: ");
  //   Serial.println(testrpm);

  //   // Reset pulse count for the next interval
  //   lastEncoderPulses = encoderValues[0];
  // }

  // Check if it's time to toggle the LED (every second)
  if (toggleLED) {
    digitalWrite(LED, !digitalRead(LED));  // Toggle the LED state
    toggleLED = false;  // Reset the flag
  }
  if (toggleMotor) {
    motorControllers[0].TurnRight(currentSpeed);
    toggleMotor = false;
    currentSpeed = (currentSpeed == 0) ? 255 : 0;
    // Serial.println("=========================================");
    Serial.println("current encoder: " + String(encoderValues[0]));
    encoderValues[0] = 0;
  }

}

void IRAM_ATTR timerIsr() {
  // This interrupt is triggered every 1 second (via timer)

  toggleLED = true;  // Set the flag to toggle the LED
  toggleMotor = true;  // Set the flag to toggle the motor

}

void setupTimerInterrupt() {
  // Create a hardware timer (Timer 0) to trigger an interrupt every 1 second (1000ms)
  timer = timerBegin(0, 80, true);  // Timer 0, 80 prescaler (1 tick = 1 microsecond)
  timerAttachInterrupt(timer, &timerIsr, true);  // Attach the ISR
  timerAlarmWrite(timer, 1000000, true);  // Set the timer to trigger every 1 second (1000000 microseconds)
  timerAlarmEnable(timer);  // Enable the timer interrupt
}

void updateEncoder1() {
  encoderValues[0]++;
}

void updateEncoder2() {
  encoderValues[1]++;
}

void updateEncoderB() {
  int B = digitalRead(ENC_IN_B);
  // Serial.println(B);
    if (B > 0) {
        posi++;
    } else {
        posi--;
    }
}
