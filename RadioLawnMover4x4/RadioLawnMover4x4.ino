#include <elapsedMillis.h>
#include "PwmReader.h"
#include "Motor.h"

#define IS_DEBUG 1
///////// PINS /////////////////////////////////////////////////////////////////////////////

// RC
#define RC_THROTTLE_PIN 19
#define RC_ARMED_PIN 20
#define RC_DIRECTION_PIN 21
#define RC_STEERING_PIN 18

// Motor Front Left
#define M1_DIR_PIN 7
#define M1_BRAKE_PIN 6
#define M1_THROTTLE_PIN 5
#define M1_SPEED_FEEDBACK_PIN 2

// Motor Front Right
#define M2_DIR_PIN 10
#define M2_BRAKE_PIN 9
#define M2_THROTTLE_PIN 8
#define M2_SPEED_FEEDBACK_PIN 3

// Motor Back Left
#define M3_DIR_PIN 42
#define M3_BRAKE_PIN 43
#define M3_THROTTLE_PIN 44
#define M3_SPEED_FEEDBACK_PIN A8

// Motor Back Right
#define M4_DIR_PIN 48
#define M4_BRAKE_PIN 47
#define M4_THROTTLE_PIN 46
#define M4_SPEED_FEEDBACK_PIN A9

//////////////////////////////////////////////////////////////////////////////////////////

#define FAILSAFE_MIN 1480
#define FAILSAFE_MAX 1496
#define CONSECUTIVE_TRIGGERS 5
#define CONSECUTIVE_RECOVERY 10
int failsafeCount = 0;

typedef enum signal_e {
  SIGNAL_THROTTLE = 0,
  SIGNAL_ARMED,
  SIGNAL_DIRECTION,
  SIGNAL_STEERING,
  SIGNAL_COUNT

} signal_e;

typedef enum steering_e {
  STEER_FULL_LEFT = 0,
  STEER_HALF_LEFT,
  STEER_CENTER,
  STEER_HALF_RIGHT,
  STEER_FULL_RIGHT,
} steering_e;

typedef enum direction_state_e {
  STOPPED = 0,
  CHANGING_DIRECTION,
  MOVING_BACKWARD,
  MOVING_FORWARD,
} direction_state_e;

volatile direction_state_e direction_state = STOPPED;
volatile direction_state_e prev_direction_state = STOPPED;
volatile bool previous_isForward = true;
volatile bool isForward = true;


const byte RC_PINS[SIGNAL_COUNT] = {
  [SIGNAL_THROTTLE] = RC_THROTTLE_PIN,
  [SIGNAL_ARMED] = RC_ARMED_PIN,
  [SIGNAL_DIRECTION] = RC_DIRECTION_PIN,
  [SIGNAL_STEERING] = RC_STEERING_PIN,
};

// Variables to store PWM values
volatile unsigned long rising_start[SIGNAL_COUNT];
volatile int pwm_values[SIGNAL_COUNT];
volatile boolean new_pwm_signal[SIGNAL_COUNT];
volatile unsigned long lastPulseTime[SIGNAL_COUNT];


volatile bool WAS_DISARMED = false;
volatile bool IS_ARMED = false;
volatile bool IS_CONNECTED = false;
volatile bool INVERSED_DIRECTION = true;
volatile int THROTTLE_VALUE = 0;
volatile int STEERING_VALUE = STEER_CENTER;


volatile int disarmed_counter = 0;
#define DISARMED_MINIMUM_SAMPLES 5

const float filter = 0.2;  // => 1 / number of samples

PwmReader throttle(filter);
PwmReader steering(filter);

Motor motorFrontLeft(M1_DIR_PIN, M1_BRAKE_PIN, M1_THROTTLE_PIN, false);
Motor motorFrontRight(M2_DIR_PIN, M2_BRAKE_PIN, M2_THROTTLE_PIN, true);
Motor motorBackLeft(M3_DIR_PIN, M3_BRAKE_PIN, M3_THROTTLE_PIN, false);
Motor motorBackRight(M4_DIR_PIN, M4_BRAKE_PIN, M4_THROTTLE_PIN, true);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

  // Configure pins and attach interrupts
  for (byte i = 0; i < SIGNAL_COUNT; i++) {
    pinMode(RC_PINS[i], INPUT);
    digitalWrite(RC_PINS[i], HIGH);
  }

  attachInterrupt(digitalPinToInterrupt(RC_THROTTLE_PIN), handle_interrupt_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_ARMED_PIN), handle_interrupt_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_DIRECTION_PIN), handle_interrupt_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_STEERING_PIN), handle_interrupt_3, CHANGE);


  throttle.setLimits(980, 2000, 1490, 20);
  throttle.setMapping(-70, 70);  // max speed 255

  steering.setLimits(880, 1700, 1380, 60);
  steering.setMapping(0, 4);

  motors_stop();
  Serial.println("Started.");
}

void printDebug() {
  if (!IS_DEBUG) return;

  Serial.print("Connected:");
  Serial.print(IS_CONNECTED);


  // Serial.print("Throttle Map:");
  // Serial.print(THROTTLE_VALUE);
  // Serial.print(", ArmedVal:");
  // Serial.print(pwm_values[1]);
  Serial.print(", Armed:");
  Serial.print(IS_ARMED);

  //Serial.print(", Direction:");
  //Serial.print(pwm_values[SIGNAL_DIRECTION]);
  //Serial.print(INVERSED_DIRECTION);
  Serial.print(", Throttle:");
  Serial.print(pwm_values[SIGNAL_THROTTLE]);

  Serial.print(", Steering:");
  // Serial.print(STEERING_VALUE);
  // Serial.print(" <- ");
  Serial.print(pwm_values[SIGNAL_STEERING]);

  // Serial.print(", WasDisArmed:");
  // Serial.print(WAS_DISARMED ? 2000 : 0);
  Serial.println();
}

bool stateHasChanged() {
  return direction_state != prev_direction_state;
}

elapsedMillis sincePrint;

void loop() {
  updateConnectionState();

  if (!IsHealthyConnection()) {
    disarm();
  } else {
    mapValues();
  }

  if (!IS_ARMED) {
    motors_brakes_on();
    return;
  }

  motors_brakes_off();

  switch (direction_state) {
    case STOPPED:
      if (stateHasChanged()) {
        handleStopOnEntry();
      } else {
        handleStop();
      }
      break;

    case CHANGING_DIRECTION:
      handleChangeDirection();
      break;

    case MOVING_BACKWARD:
    case MOVING_FORWARD:
      handleMoving();
      break;
  }


  if (sincePrint > 1000) {
    sincePrint = 0;
    printDebug();
  }

  delay(20);
}

void handleChangeDirection() {
  motors_stop();
  Serial.println("Stopped");
  delay(500);

  Serial.print("Moving ");
  if (isForward) {
    Serial.println("forward");
  } else {
    Serial.println("backward");
  }
  // Change direction

  start_moving();
  // Change state to moving
  if (isForward) {
    changeState(MOVING_FORWARD);
  } else {
    changeState(MOVING_BACKWARD);
  }
}

void handleMoving() {

  if (THROTTLE_VALUE == 0) {
    changeState(STOPPED);
    return;
  }

  isForward = THROTTLE_VALUE > 0;
  if (previous_isForward != isForward) {
    previous_isForward = isForward;
    changeState(CHANGING_DIRECTION);
    return;
  }

  uint8_t speed = abs(THROTTLE_VALUE);
  switch (STEERING_VALUE) {
    case STEER_FULL_LEFT:
      motorFrontLeft.setSpeed(speed, !isForward);
      motorFrontRight.setSpeed(speed, isForward);
      motorBackLeft.setSpeed(speed, !isForward);
      motorBackRight.setSpeed(speed, isForward);
      break;
    case STEER_HALF_LEFT:
      motorFrontLeft.setSpeed(speed, 0);
      motorFrontRight.setSpeed(speed, isForward);
      motorBackLeft.setSpeed(speed, 0);
      motorBackRight.setSpeed(speed, isForward);
      break;
    case STEER_CENTER:
      motorFrontLeft.setSpeed(speed, isForward);
      motorFrontRight.setSpeed(speed, isForward);
      motorBackLeft.setSpeed(speed, isForward);
      motorBackRight.setSpeed(speed, isForward);
      break;
    case STEER_HALF_RIGHT:
      motorFrontLeft.setSpeed(speed, isForward);
      motorFrontRight.setSpeed(speed, 0);
      motorBackLeft.setSpeed(speed, isForward);
      motorBackRight.setSpeed(speed, 0);
      break;
    case STEER_FULL_RIGHT:
      motorFrontLeft.setSpeed(speed, isForward);
      motorFrontRight.setSpeed(speed, !isForward);
      motorBackLeft.setSpeed(speed, isForward);
      motorBackRight.setSpeed(speed, !isForward);
      break;
  }

  previous_isForward = isForward;
}

void handleStopOnEntry() {
  motors_stop();
  prev_direction_state = direction_state;
  delay(500);
  Serial.println("Stopped (on entry)");
}

void changeState(direction_state_e newState) {
  prev_direction_state = direction_state;
  direction_state = newState;
  Serial.print("Old state: ");
  //printState(prev_direction_state, true);
  Serial.print("Change State: ");
  //printState(direction_state, true);
}

void start_moving() {
  // do nothing or maybe remove brakes only here?
}

void handleStop() {

  previous_isForward = THROTTLE_VALUE > 0;

  if (IS_ARMED && THROTTLE_VALUE != 0) {
    Serial.print("Throttle: ");
    Serial.println(THROTTLE_VALUE);
    start_moving();

    if (isForward) {
      changeState(MOVING_FORWARD);
    } else {
      changeState(MOVING_BACKWARD);
    }
  }
  // else {
  //   Serial.print("handleStop: ");
  //   Serial.print(IS_ARMED);
  //   Serial.print(", Throttle: ");
  //   Serial.println(THROTTLE_VALUE);
  // }
}

void motors_stop() {
  motorFrontLeft.stop();
  motorFrontRight.stop();
  motorBackLeft.stop();
  motorBackRight.stop();
}

void motors_brakes_on() {
  motorFrontLeft.brakes_on();
  motorFrontRight.brakes_on();
  motorBackLeft.brakes_on();
  motorBackRight.brakes_on();
}

void motors_brakes_off() {
  motorFrontLeft.brakes_off();
  motorFrontRight.brakes_off();
  motorBackLeft.brakes_off();
  motorBackRight.brakes_off();
}

bool isFailsafeSignature() {
  unsigned long pwA, pwB;

  noInterrupts();
  pwA = pwm_values[SIGNAL_THROTTLE];
  pwB = pwm_values[SIGNAL_STEERING];
  interrupts();

  return (pwA >= FAILSAFE_MIN && pwA <= FAILSAFE_MAX && pwB >= FAILSAFE_MIN && pwB <= FAILSAFE_MAX);
}

void updateConnectionState() {
  if (isFailsafeSignature()) {
    failsafeCount = min(failsafeCount + 1, CONSECUTIVE_TRIGGERS);  // cap at max
  } else {
    failsafeCount = max(failsafeCount - 1, -CONSECUTIVE_RECOVERY);  // cap at min
  }
}

bool isConnectionLost() {
  return failsafeCount >= CONSECUTIVE_TRIGGERS;
}

bool IsHealthyConnection() {
  IS_CONNECTED = failsafeCount <= -CONSECUTIVE_RECOVERY;
  return IS_CONNECTED;
}

void disarm() {
  WAS_DISARMED = false;
  IS_ARMED = false;
}

void mapValues() {
  noInterrupts();
  for (int signal = 0; signal < SIGNAL_COUNT; signal++) {
    if (pwm_values[signal] <= 0) {
      continue;
    }

    switch (signal) {
      case SIGNAL_DIRECTION:
        INVERSED_DIRECTION = pwm_values[signal] < 1800;
        break;
      case SIGNAL_ARMED:
        set_isarmed(pwm_values[signal]);
        break;
      case SIGNAL_THROTTLE:
        THROTTLE_VALUE = throttle.mapTo(pwm_values[signal]);
        if (INVERSED_DIRECTION) {
          THROTTLE_VALUE *= -1;
        }
        break;
      case SIGNAL_STEERING:
        STEERING_VALUE = steering.mapTo(pwm_values[signal]);
        break;
    }
  }
  interrupts();
}

void handle_interrupt_0() {
  handle_interrupt(SIGNAL_THROTTLE);
}

void handle_interrupt_1() {
  handle_interrupt(SIGNAL_ARMED);
}

void handle_interrupt_2() {
  handle_interrupt(SIGNAL_DIRECTION);
}

void handle_interrupt_3() {
  handle_interrupt(SIGNAL_STEERING);
}

void set_isarmed(int val) {
  bool is_armed = val > 1900 && val < 3000;

  if (is_armed && WAS_DISARMED) {
    IS_ARMED = true;
    disarmed_counter = 0;
    return;
  }

  disarmed_counter++;

  if (disarmed_counter >= DISARMED_MINIMUM_SAMPLES) {
    IS_ARMED = false;

    if (val > 0 && val < 1100) {
      WAS_DISARMED = true;
    }
  }
}

void handle_interrupt(byte signal) {
  uint8_t trigger = digitalRead(RC_PINS[signal]);
  unsigned long current_time = micros();

  if (trigger == HIGH) {
    rising_start[signal] = current_time;
  } else {
    // Calculate pulse width on falling edge
    if (rising_start[signal] != 0) {
      pwm_values[signal] = (int)(current_time - rising_start[signal]);
      new_pwm_signal[signal] = true;

      // Sanity check for typical RC values (usually 1000-2000μs)
      if (pwm_values[signal] < 800 || pwm_values[signal] > 2100) {
        pwm_values[signal] = 0;  // Invalid signal
      }

      lastPulseTime[signal] = millis();
    }
  }
}