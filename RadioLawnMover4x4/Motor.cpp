#include <Arduino.h>
#include "Motor.h"
#include <Ewma.h>

Motor::Motor(uint8_t directionPin, uint8_t brakePin, uint8_t throttlePin, bool reversedDirection) : adcFilter(0.2) {
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
  speed = adcFilter.filter(speed); 
  analogWrite(_throttlePin, speed);
  digitalWrite(_directionPin, isForward ^ _reversedDirection);
  // Serial.print("setSpeed: ");
  // Serial.println(speed);
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