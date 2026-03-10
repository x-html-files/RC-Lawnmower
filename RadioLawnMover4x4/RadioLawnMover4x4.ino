#include "PwmReader.h"
#include <elapsedMillis.h>

#define IS_DEBUG 1

#define RC_THROTTLE_PIN 19
#define RC_ARMED_PIN 20
#define RC_DIRECTION_PIN 21
#define RC_STEERING_PIN 18

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

elapsedMillis sincePrint;

void loop() {
  updateConnectionState();

  if (!IsHealthyConnection()) {
    disarm();
  } else {
    mapValues();
  }

  if (sincePrint > 1000) {
      sincePrint = 0;
      printDebug();
    }

  delay(20);
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