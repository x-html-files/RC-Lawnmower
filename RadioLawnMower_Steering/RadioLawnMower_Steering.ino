#include <AccelStepper.h>
#include <Ewma.h>
#include <Wire.h>
#include "PwmReader.h"

#define IS_DEBUG 0

// STEPPER MOTOR
#define motorInterfaceType 1
#define MICROSTEPS 2
#define DIRECTION_PIN 5
#define STEP_PIN 6

// IO PINS
#define SENSOR_RIGHT 10
#define SENSOR_CENTER 11
#define SENSOR_LEFT 12
#define RC_DIRECTION_PIN 2

// ENUMS
#define DEST_STOP 0
#define DEST_LEFT 1
#define DEST_CENTER 2
#define DEST_RIGHT 3

#define DIR_STOP 0
#define DIR_LEFT 1
#define DIR_RIGHT 3

#define POS_UNKNOWN 0
#define POS_LEFT 1
#define POS_CENTER 2
#define POS_RIGHT 3

#define STEERING_CHL 0      // Channel 1

#define WIRE_SLAVE_ID 6

const long LONG_MAX = 2147483646;
const long LONG_MIN = -2147483647;

volatile int position = POS_UNKNOWN;
volatile int destination = DEST_STOP;

volatile int currentDir = DIR_STOP;
volatile int prevDir = DIR_STOP;

const byte NUM_CHANNELS = 1;
const byte RC_PINS[] = { RC_DIRECTION_PIN };

// Variables to store PWM values
volatile unsigned long rising_start[NUM_CHANNELS];
volatile int pwm_values[NUM_CHANNELS];
volatile boolean new_pwm_signal[NUM_CHANNELS];

// Creates an instance
AccelStepper myStepper(motorInterfaceType, STEP_PIN, DIRECTION_PIN);

unsigned long previousMillis = 0;
unsigned long interval = 1000;

const float STEPPER_SPEED = 500 * MICROSTEPS; // was 1000
const float STEPPER_ACCEL = 2000 * MICROSTEPS;


volatile int STEERING_VALUE = 0;

const float filter = 0.1;  // => 1 / number of samples
PwmReader steering(filter);

volatile bool IS_ARMED = false;
unsigned long last_armed_millis;
unsigned long max_no_transfer_millis = 3000;

void setup() {

  Serial.begin(115200);
  myStepper.setMaxSpeed(STEPPER_SPEED);
  myStepper.setAcceleration(STEPPER_ACCEL);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(RC_DIRECTION_PIN), handle_interrupt_0, CHANGE);

  steering.setLimits(930, 948, 1501, 1450, 50);
  steering.setMapping(0, 2);

  // Start the I2C Bus as Slave on address 9
  Wire.begin(WIRE_SLAVE_ID);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}

char command = 0;
void receiveEvent(int bytes) {
  command = Wire.read();    // read one character from the I2C
  
  if(command == 'a')
  {
    IS_ARMED = true;
    last_armed_millis = millis();
  }else
  {
    IS_ARMED = false;
  }

  if(IS_DEBUG)
  {
    Serial.print("command: ");
    Serial.println(command);
  }
}

volatile int potStep = 0;
volatile int potStepPrev = 0;
volatile int sensor_prev = 0;
volatile bool running = true;


void handle_interrupt_0() {
  handle_interrupt(0);
}

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
        case STEERING_CHL:
          STEERING_VALUE = pwm_values[channel];
          break;
      }
    }
  }
}

int mapDestination(int input) {

  int v = steering.mapTo(input);

  switch(v)
  {
    case 0:
      return DEST_LEFT;
    case 2:
      return DEST_RIGHT;
    default:
      return DEST_CENTER;
  }
}

void setDirectionLeft() {
  Serial.print("setDirectionLeft: from 0 to ");
  Serial.println(LONG_MIN);

  myStepper.setCurrentPosition(0);
  myStepper.moveTo(LONG_MIN);
}

void setDirectionRight() {
  Serial.print("setDirectionRight: from 0 to ");
  Serial.println(LONG_MAX);

  myStepper.setCurrentPosition(0);
  myStepper.moveTo(LONG_MAX);
}

String defToString(int destination) {
  switch (destination) {
    case 1:
      return "LEFT";
    case 2:
      return "CENTER";
    case 3:
      return "RIGHT";
    default:
      return String(destination);
  }
}

volatile int prev_sensor_center = 0;
volatile int prev_sensor_trig = 0;
volatile int sensor_trig = 0;

volatile int sensor_right;
volatile int sensor_left;
volatile int sensor_center;

void read_stopswitches()
{
  sensor_right = digitalRead(SENSOR_LEFT);
  sensor_left = digitalRead(SENSOR_RIGHT);
  sensor_center = digitalRead(SENSOR_CENTER);

  sensor_trig = (!sensor_right || !sensor_left);

  if (sensor_trig != prev_sensor_trig) {
    if (sensor_trig) {
      digitalWrite(LED_BUILTIN, HIGH);
      if (!sensor_right) {
        Serial.println("Limit switch RIGHT triggered");
      }
      if (!sensor_left) {
        Serial.println("Limit switch LEFT triggered");
      }
    } else {
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Limit switches untriggered");
    }
  }

  prev_sensor_trig = sensor_trig;
}

bool is_armed()
{
  if(!IS_ARMED)
  {
    return false;
  }

  unsigned long currentMillis = millis();
  if (currentMillis - last_armed_millis > max_no_transfer_millis) 
  {
    IS_ARMED = false;
    return false;
  }

  return true;
}

void loop() {

  printDebug();

  if(!is_armed())
  {
    myStepper.stop();
    return;
  }

  read_stopswitches();

  destination = mapDestination(STEERING_VALUE);

  switch (destination) {
    case DEST_STOP:
      myStepper.stop();
      break;

    case DEST_RIGHT:
      currentDir = DIR_RIGHT;

      if (!sensor_right) {
        position = POS_RIGHT;
        currentDir = DIR_STOP;
        myStepper.stop();
        break;
      }

      if (!prev_sensor_center && sensor_center) {
        position = POS_RIGHT;
      }

      if (prevDir == currentDir) {
        myStepper.run();
      } else {
        myStepper.stop();
        setDirectionRight();
      }

      break;

    case DEST_LEFT:

      currentDir = DIR_LEFT;

      if (!sensor_left) {
        position = POS_LEFT;
        currentDir = DIR_STOP;
        myStepper.stop();
        break;
      }

      if (!prev_sensor_center && sensor_center) {
        position = POS_LEFT;
      }

      if (prevDir == currentDir) {
        myStepper.run();
      } else {
        setDirectionLeft();
      }
      break;

    case DEST_CENTER:

      if (!sensor_center) {
        position = POS_CENTER;
        currentDir = DIR_STOP;
        //myStepper.disableOutputs();
        myStepper.stop();
        break;
      }

      if (!sensor_left) {
        position = POS_LEFT;
      }

      if (!sensor_right) {
        position = POS_RIGHT;
      }

      if (position == POS_CENTER) {
        position = POS_UNKNOWN;
      }

      // If on the left - Go right
      if (position == POS_LEFT) {
        if (currentDir == DIR_RIGHT) {
          myStepper.run();

        } else {
          currentDir = DIR_RIGHT;
          setDirectionRight();
        }
        break;
      }

      // If on the right or don't know - go left
      if (currentDir == DIR_LEFT) {
        myStepper.run();
      } else {
        currentDir = DIR_LEFT;
        setDirectionLeft();
      }
      break;
  }

  prevDir = currentDir;
  prev_sensor_center = sensor_center;
}

void printDebug()
{
  if(!IS_DEBUG) return;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;

      Serial.print("Armed: ");

    if (IS_ARMED) {
      Serial.print("YES -> ");
    } else {
      Serial.print("NO -> ");
    }
    Serial.print(last_armed_millis);
    long c = millis();
    Serial.print(", Interval: ");
    Serial.println(c - last_armed_millis);

    Serial.print(", Destination: ");
    Serial.print(defToString(destination));
    Serial.print(", Direction: ");
    Serial.print(defToString(currentDir));
    Serial.print(", Position: ");
    Serial.print(defToString(position));
    Serial.println();
  }
}