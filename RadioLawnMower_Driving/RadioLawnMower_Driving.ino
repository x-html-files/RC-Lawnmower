
#include <Ewma.h>
#include <Wire.h>
#include "PwmReader.h"

#define IS_DEBUG 0

enum MODE_TYPE {
  TURN_BY_WHEELS,
  TURN_BY_STEPPER
};

const MODE_TYPE MODE = TURN_BY_STEPPER;

#define THROTTLE 0  // Channel 1
#define ARMED 1     // Channel 4
#define DIRECTION 2     // Channel 5

#define RC_SPEED_PIN 2
#define RC_ARMED_PIN 3
#define RC_DIRECTION_PIN 4

#define LEFT_MOTOR_PIN 6
#define RIGHT_MOTOR_PIN 5
#define LEFT_DIRECTION_PIN 8
#define RIGHT_DIRECTION_PIN 7
#define STOP_PIN 9

#define WIRE_SLAVE_ID 6
unsigned long wire_prev_millis;
unsigned long wire_transfer_interval = 200;

const byte RC_PINS[] = { RC_SPEED_PIN, RC_ARMED_PIN, RC_DIRECTION_PIN };  // Arduino Uno digital pins // 5, 6, 7
const byte NUM_CHANNELS = 3;

// Variables to store PWM values
volatile unsigned long rising_start[NUM_CHANNELS];
volatile int pwm_values[NUM_CHANNELS];
volatile boolean new_pwm_signal[NUM_CHANNELS];

volatile int THROTTLE_VALUE = 0;
volatile int ARMED_VALUE = 0;
volatile bool IS_ARMED = false;
volatile bool INVERSED_DIRECTION = true;
volatile int DIRECTION_VALUE = 0;

const float filter = 0.1;  // => 1 / number of samples

PwmReader throttle(filter);


enum State {
  STOPPED,             // Value 0
  CHANGING_DIRECTION,  // Value 1
  MOVING_BACKWARD,
  MOVING_FORWARD,
};

volatile State state = STOPPED;
volatile State prev_state = STOPPED;
volatile bool previous_isForward = true;
volatile bool isForward = true;

//long previousMillis = 0;

void setup() {
  Serial.begin(115200);
  // Start the I2C Bus as Master
  Wire.begin();

  // Configure pins and attach interrupts
  for (byte i = 0; i < NUM_CHANNELS; i++) {
    pinMode(RC_PINS[i], INPUT);
    // Enable internal pullup to prevent floating inputs
    digitalWrite(RC_PINS[i], HIGH);
  }

  // Hardware interrupts for Uno pins 2 and 3
  attachInterrupt(digitalPinToInterrupt(2), handle_interrupt_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), handle_interrupt_1, CHANGE);

  // Pin Change Interrupts for Uno pins 4-7 (PORTD)
  PCICR |= (1 << PCIE2);     // Enable PCINT for PORTD
  PCMSK2 |= (1 << PCINT20);  // Pin 4
  // PCMSK2 |= (1 << PCINT21); // Pin 5
  // PCMSK2 |= (1 << PCINT22); // Pin 6
  // PCMSK2 |= (1 << PCINT23); // Pin 7

  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);
  pinMode(STOP_PIN, OUTPUT);

  // speed
  throttle.setLimits(980, 980, 2000, 1490, 20);
  throttle.setMapping(-70, 70);  // max speed 255
}

void stop_motors() {
  digitalWrite(STOP_PIN, LOW);
  //Serial.println("STOP_PIN LOW");
  analogWrite(LEFT_MOTOR_PIN, 0);
  analogWrite(RIGHT_MOTOR_PIN, 0);
}

void start_moving() {
  digitalWrite(STOP_PIN, HIGH);
  //Serial.println("STOP_PIN HIGH");
}

void changeState(State newState) {
  prev_state = state;
  state = newState;
  Serial.print("Old state: ");
  printState(prev_state, true);
  Serial.print("Change State: ");
  printState(state, true);
}

void printState(State state, bool newline) {
  switch (state) {
    case STOPPED:
      Serial.print("STOPPED");
      break;
    case CHANGING_DIRECTION:
      Serial.print("CHANGING_DIRECTION");
      break;
    case MOVING_BACKWARD:
      Serial.print("MOVING_BACKWARD");
      break;
    case MOVING_FORWARD:
      Serial.print("MOVING_FORWARD");
      break;
  }

  if (newline) {
    Serial.println();
  }
}

bool stateHasChanged() {
  return state != prev_state;
}


void handleStopOnEntry() {
  stop_motors();
  prev_state = state;
  delay(500);
  Serial.println("Stopped (on entry)");
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

void handleChangeDirection() {
  stop_motors();
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

  digitalWrite(LEFT_DIRECTION_PIN, isForward);
  digitalWrite(RIGHT_DIRECTION_PIN, !isForward);

  analogWrite(LEFT_MOTOR_PIN, abs(THROTTLE_VALUE));
  analogWrite(RIGHT_MOTOR_PIN, abs(THROTTLE_VALUE));

  previous_isForward = isForward;
}


int count = 0;
unsigned long msecLst;

#define TWO_SEC 2000

void loop() {

  printDebug();

  sendWire();

  if (!IS_ARMED) {
    stop_motors();
    return;
  }

  switch (state) {
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
}


// Hardware interrupt handlers for pins 2 and 3
void handle_interrupt_0() {
  handle_interrupt(0);
}

void handle_interrupt_1() {
  handle_interrupt(1);
}

// Pin Change Interrupt handler for PORTD (pins 4-7)
ISR(PCINT2_vect) {
  static uint8_t prev_port_state = PIND;
  uint8_t changed_bits = PIND ^ prev_port_state;
  prev_port_state = PIND;

  // Check which pins changed
  for (byte i = 2; i < NUM_CHANNELS; i++) {
    if (changed_bits & (1 << RC_PINS[i])) {
      handle_interrupt(i);
    }
  }
}

// Generic interrupt handler
void handle_interrupt(byte channel) {
  uint8_t trigger = digitalRead(RC_PINS[channel]);
  unsigned long current_time = micros();

  if (trigger == HIGH) {
    rising_start[channel] = current_time;
  } else {
    // Calculate pulse width on falling edge
    if (rising_start[channel] != 0) {
      pwm_values[channel] = (int)(current_time - rising_start[channel]);
      new_pwm_signal[channel] = true;

      // Sanity check for typical RC values (usually 1000-2000μs)
      if (pwm_values[channel] < 900 || pwm_values[channel] > 2100) {
        pwm_values[channel] = 0;  // Invalid signal
      }

      switch (channel) {
        case DIRECTION:
          DIRECTION_VALUE = pwm_values[channel];
          INVERSED_DIRECTION = DIRECTION_VALUE < 1900 || DIRECTION_VALUE > 2200;

        case ARMED:
          ARMED_VALUE = pwm_values[channel];  //pitch.mapTo(pwm_values[channel]);
          IS_ARMED = ARMED_VALUE > 1900 && ARMED_VALUE < 3000;

          break;
        case THROTTLE:
          THROTTLE_VALUE = throttle.mapTo(pwm_values[channel]);

          if(INVERSED_DIRECTION)
          {
            THROTTLE_VALUE *= -1;
          }
          break;
      }
    }
  }
}

void sendWire()
{
  unsigned long msec = millis();

  if ((msec - wire_prev_millis) > wire_transfer_interval) {
    wire_prev_millis = msec;

    char command;
    if (IS_ARMED) {
      command = 'a';
    } else {
      command = 'u';
    }
    
    Wire.beginTransmission(WIRE_SLAVE_ID); // transmit to device #9
    Wire.write(command);              // sends x 
    Wire.endTransmission();    // stop transmitting

    if(IS_DEBUG)
    {
      Serial.print("Direction: ");
      Serial.print(DIRECTION_VALUE);  

      Serial.print("command: ");
      Serial.print(command);
      Serial.print(", armed: ");

      if (IS_ARMED) {
        Serial.print("YES -> ");
      } else {
        Serial.print("NO -> ");
      }
      Serial.println(ARMED_VALUE);
    }
  }
}

void printDebug()
{
  if(!IS_DEBUG) return;

  unsigned long msec = millis();

  if ((msec - msecLst) > TWO_SEC) {
    msecLst = msec;
    count++;
    //Serial.println (count);

    Serial.print("state: ");
    printState(state, false);

    Serial.print(", throttle: ");
    Serial.print(THROTTLE_VALUE);

    Serial.print(", direction: ");
    if (isForward) {
      Serial.print("FORWARD");
    } else {
      Serial.print("BACKWARD");
    }

    Serial.print(", armed: ");

    if (IS_ARMED) {
      Serial.print("YES -> ");
    } else {
      Serial.print("NO -> ");
    }
    Serial.println(ARMED_VALUE);
  }
}
