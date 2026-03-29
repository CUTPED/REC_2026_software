#include <Arduino.h>
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

#include "Motor_PID.h"
#include "Control_Panel.h"

// ─── Pin Definitions ─────────────────────────────────────────────────────────
#define ENABLE_PIN 12

#define LIFT_MOTOR_ENCODER_A 34
#define LIFT_MOTOR_ENCODER_B 35
#define LIFT_MOTOR_PWM_1 32
#define LIFT_MOTOR_PWM_2 33
#define SHUTDOWN_PIN 4

#define LIFT_MOTOR_CPR 7974.4
#define LIFT_MOTOR_KP 50.0f
#define LIFT_MOTOR_KI 0.0f
#define LIFT_MOTOR_KD 0.5f

#define MAX_LIFT_POS 900.f // Maximum position for the lift motor in degrees /4

MotorPID LiftMotor;
volatile float GOAL_POS = 0.0f; // Target position for the lift motor in degrees /4
volatile float internal_timer = 0.0f;

enum RideState
{
  SPIN_UP,
  HOLD_MAX,
  SPIN_DOWN,
  HOLD_STOP
};
RideState rideState = SPIN_UP;
unsigned long stateStartTime = 0;
bool rideStarted = false;

const unsigned long SPIN_UP_TIME = 20000; // ms
const unsigned long HOLD_MAX_TIME = 20000;
const unsigned long SPIN_DOWN_TIME = 20000;
const unsigned long HOLD_STOP_TIME = 24000;
volatile long now = 0;

ControlPanel panel;


void loop()
{
  unsigned long yesterday = now;
  now = millis();

  // Initialize timer on first loop
  if (!rideStarted)
  {
    stateStartTime = now;
    // rideStarted = true;
  }else{

  unsigned long elapsed = now - stateStartTime;

  switch (rideState)
  {

  case SPIN_UP:
    digitalWrite(SHUTDOWN_PIN, HIGH);
    GOAL_POS += MAX_LIFT_POS/SPIN_UP_TIME * (now-yesterday); // Update goal position based on time elapsed
    if (GOAL_POS > MAX_LIFT_POS)
      GOAL_POS = MAX_LIFT_POS;

    if (elapsed >= SPIN_UP_TIME)
    {
      GOAL_POS = MAX_LIFT_POS; // Ensure we're exactly at max
      rideState = HOLD_MAX;
      stateStartTime = now;
    }
    break;

  case HOLD_MAX:
    GOAL_POS = MAX_LIFT_POS;

    if (elapsed >= HOLD_MAX_TIME)
    {
      rideState = SPIN_DOWN;
      stateStartTime = now;
    }
    break;

  case SPIN_DOWN:
    GOAL_POS -= MAX_LIFT_POS/SPIN_DOWN_TIME * (now-yesterday); // Update goal position based on time elapsed
     if (GOAL_POS < 0.0f)
       GOAL_POS = 0.0f;

    if (elapsed >= SPIN_DOWN_TIME)
    {
      GOAL_POS = 0.0f; // Ensure we're exactly at zero
      rideState = HOLD_STOP;
      stateStartTime = now;
    }
    break;

  case HOLD_STOP:
    GOAL_POS = 0.0f;
    digitalWrite(SHUTDOWN_PIN, LOW);
    if (elapsed >= HOLD_STOP_TIME)
    {
      // Ride complete — restart or just stay stopped
      rideState = SPIN_UP; // Remove this line to stop permanently
      stateStartTime = now;
    }
    break;
  }
  LiftMotor.setGoalPos(GOAL_POS);
  LiftMotor.update();

  // Serial.print(" State: ");
  // Serial.print(rideState);
  Serial.print(" Goal Position: ");
  Serial.print(GOAL_POS);
  Serial.print(" Motor Position: ");
  Serial.println(LiftMotor.getPos());
  // Serial.print(" Motor RPM: ");
  // Serial.print(LiftMotor.getRPM());
  // Serial.print(" Motor PWM: ");
  // Serial.println(LiftMotor._currentPWM);
  // Serial.println(" Time Elapsed: ");
  // Serial.println(elapsed);
}
  delay(10);
}

// void IRAM_ATTR onInput (uint16_t input) {
  Serial.print("Input received: ");
  Serial.println(input, BIN);
  if(!(input & 1<<4)) { 
    rideStarted = true;
  }

}

void setup()
{
  panel.init();
  panel.setInputCallback(onInput);
  LiftMotor.init(LIFT_MOTOR_ENCODER_A, LIFT_MOTOR_ENCODER_B, LIFT_MOTOR_PWM_1, LIFT_MOTOR_PWM_2, ENABLE_PIN, LIFT_MOTOR_CPR, LEDC_CHANNEL_6, LEDC_CHANNEL_7, LIFT_MOTOR_KP, LIFT_MOTOR_KI, LIFT_MOTOR_KD, 10);
  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW); // Ensure the motor driver is enabled by setting the shutdown pin low
                                    //   pinMode(LIFT_MOTOR_PWM_1, OUTPUT);
                                    //   pinMode(LIFT_MOTOR_PWM_2, OUTPUT);
                                    //   digitalWrite(LIFT_MOTOR_PWM_2, LOW);

  //   Motor1.setGoalVelo(135.0f);
  //   Motor2.setGoaslVelo(135.0f);
  //   Motor3.setGoalVelo(135.0f);
  // LiftMotor.setGoalPos(-100.0f); // Just for testing, set a goal position of 360 degrees (1 full rotation) for the lift motor
  LiftMotor.enable();
  Serial.begin(115200);

  // pinMode(ENABLE_PIN, OUTPUT);
  // digitalWrite(ENABLE_PIN, HIGH); // Enable the motor driver by setting the enable pin high

  // pinMode(MOTOR_1_PWM_1, OUTPUT);
  // pinMode(MOTOR_1_PWM_2, OUTPUT);

  // analogWrite(MOTOR_1_PWM_2, 127); // Set initial direction to low
  // digitalWrite(MOTOR_1_PWM_1, LOW); // Enable the motor driver by setting the enable pin high

  // pinMode(MOTOR_2_PWM_1, OUTPUT);
  // pinMode(MOTOR_2_PWM_2, OUTPUT);

  // analogWrite(MOTOR_2_PWM_2, 127); // Set initial direction to low
  // digitalWrite(MOTOR_2_PWM_1, LOW); // Enable the motor driver by setting the enable pin high

  // pinMode(MOTOR_3_PWM_1, OUTPUT);
  // pinMode(MOTOR_3_PWM_2, OUTPUT);

  // analogWrite(MOTOR_3_PWM_2, 127); // Set initial direction to low
  // digitalWrite(MOTOR_3_PWM_1, LOW); // Enable the motor driver by setting the enable pin high
}
