#include <Arduino.h>
#include "Motor.h"

Motor::Motor(uint8_t directionPin, uint8_t brakePin, uint8_t throttlePin, bool reversedDirection) {
  _directionPin = directionPin;
  _brakePin = brakePin;
  _throttlePin = throttlePin;
  _reversedDirection = reversedDirection;

  pinMode(_directionPin, OUTPUT);
  pinMode(_brakePin, OUTPUT);
  pinMode(_throttlePin, OUTPUT);

}

void Motor::setSpeed(uint8_t speed, bool isForward) {
  //digitalWrite(_brakePin, 0);
  analogWrite(_throttlePin, speed);
  digitalWrite(_directionPin, isForward ^ _reversedDirection);
}

void Motor::brakes_on()
{
  digitalWrite(_brakePin, 1);
}

void Motor::brakes_off()
{
  digitalWrite(_brakePin, 0);
}

void Motor::stop()
{
  analogWrite(_throttlePin, 0);
}