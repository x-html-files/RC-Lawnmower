#include <AccelStepper.h>
#include "PwmReader.h"
#include <Ewma.h>

// create an instance of the stepper class using the steps and pins

const int dirPin = 5;
const int stepPin = 6;

#define motorInterfaceType 1
#define MICROSTEPS 1

#define SENSOR_RIGHT 10
#define SENSOR_CENTER 11
#define SENSOR_LEFT 12
#define POT_PIN A4
#define RC_PIN 2


#define POT_THRESHOLD 10  // Threshold for potentiometer value change
#define STABLE_TIME 200   // Time in ms to consider value stable

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

const long LONG_MAX = 2147483646;
const long LONG_MIN = -2147483647;

volatile int position = POS_UNKNOWN;
volatile int destination = DEST_STOP;

volatile int currentDir = DIR_STOP;
volatile int prevDir = DIR_STOP;

const byte RC_PINS[] = { 2 };
const byte NUM_CHANNELS = 1;

// Variables to store PWM values
volatile unsigned long rising_start[NUM_CHANNELS];
volatile int pwm_values[NUM_CHANNELS];
volatile boolean new_pwm_signal[NUM_CHANNELS];

#define STEERING 0      // Channel 1


float motorMaxSpeed = 400.0 * MICROSTEPS;
float motorSpeed = 200.0 * MICROSTEPS;  //pulse per second
float motorAccel = 19200;

int maxDistance = 200 * MICROSTEPS * 8;
int middlePoint = maxDistance / 2;

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

unsigned long previousMillis = 0;
unsigned long interval = 2000;
int a = 0;

volatile int Pval = 0;

volatile int potVal = 0;
volatile int Val = 0;

// Variables for pot reading stabilization
int lastStablePotValue = 0;

unsigned long lastChangeTime = 0;
bool isFirstReading = true;

const float speed = 500; // was 1000
const float accel = 2000;

volatile int STEERING_VALUE = 0;

const float filter = 0.1;  // => 1 / number of samples

PwmReader steering(filter);

void setup() {

  Serial.begin(115200);
  myStepper.setMaxSpeed(speed);
  myStepper.setAcceleration(accel);
  //myStepper.setSpeed(0);
  //myStepper.setAcceleration(motorAccel);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_CENTER, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(RC_PIN), handle_interrupt_0, CHANGE);

  steering.setLimits(930, 948, 1501, 1450, 50);
  steering.setMapping(0, 2);
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
        case STEERING:
          STEERING_VALUE = pwm_values[channel]; //roll.mapTo(pwm_values[channel]);
          break;
      }
    }
  }
}

int mapDestination(int input) {

  int v = steering.mapTo(input);

  // Serial.print("val: ");
  // Serial.print(input);
  // Serial.print(", map: ");
  // Serial.println(v);

  switch(v)
  {
    case 2:
      return DEST_LEFT;
    case 0:
      return DEST_RIGHT;
    default:
      return DEST_CENTER;
  }

  // Val = map(input, 0, 350, 0, 100);



  // int d = DEST_CENTER;

  // if (Val < 40) {
  //   d = DEST_LEFT;
  // } else if (Val > 60) {
  //   d = DEST_RIGHT;
  // }

  // return d;
}

void setDirectionLeft() {
  //int speedl = speed * -1;
  Serial.print("setDirectionLeft: from 0 to ");
  Serial.println(LONG_MIN);
  myStepper.setCurrentPosition(0);
  myStepper.moveTo(LONG_MIN);
}

void setDirectionRight() {
  //int speedl = speed;
  Serial.print("setDirectionRight: from 0 to ");
  Serial.println(LONG_MAX);
  //myStepper.setSpeed(speedl);
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

void loop() {

  int sensor_right = digitalRead(SENSOR_LEFT);
  int sensor_left = digitalRead(SENSOR_RIGHT);
  int sensor_center = digitalRead(SENSOR_CENTER);

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

  //potVal = getSmartPotValue(); //analogRead(A1);

  // Serial.print("Steering ");
  // Serial.println(STEERING_VALUE);

  // delay(20);
  // return;

  // if (Serial.available() > 0) {

  //   potVal = Serial.readString().toInt();
  //   Serial.print("Received input: ");
  //   Serial.println(potVal);
  //   destination = mapDestination(potVal);
  // }

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

  // myStepper.moveTo(Val);

  // Pval = potVal;
  // potStepPrev = potStep;

  // if (myStepper.distanceToGo())
  // {
  //     myStepper.run();
  // }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    a++;
    previousMillis = currentMillis;
    Serial.print("Pot: ");
    Serial.print(potVal);
    Serial.print(", Destination: ");
    Serial.print(defToString(destination));
    Serial.print(", Direction: ");
    Serial.print(defToString(currentDir));
    Serial.print(", Position: ");
    Serial.print(defToString(position));
    Serial.println();
  }
}

int getSmartPotValue() {
  int rawValue = analogRead(POT_PIN);
  unsigned long currentTime = millis();

  // Initialize on first reading
  if (isFirstReading) {
    lastStablePotValue = rawValue;
    lastChangeTime = currentTime;
    isFirstReading = false;
    return rawValue;
  }

  // If movement detected (change > threshold), return raw value immediately
  if (abs(rawValue - lastStablePotValue) > POT_THRESHOLD) {

    lastChangeTime = currentTime;   // Reset stability timer
    lastStablePotValue = rawValue;  // Update stable value
    return rawValue;                // Return immediate raw value
  }

  // If no significant movement, check if we've been stable long enough
  if (currentTime - lastChangeTime >= STABLE_TIME && abs(rawValue - lastStablePotValue) <= POT_THRESHOLD) {
    // After stable period, keep the last stable value
    return lastStablePotValue;
  }
  // During the 500ms stabilization period, return raw value
  return rawValue;
}
