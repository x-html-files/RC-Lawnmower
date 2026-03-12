#ifndef Motor_h
#define Motor_h

class Motor {
public:
  Motor(uint8_t directionPin, uint8_t brakePin, uint8_t throttlePin, bool reversedDirection);
  void setSpeed(uint8_t speed, bool isForward);
  void brakes_on();
   void brakes_off();
  void stop();
private:
  uint8_t _directionPin;
  uint8_t _brakePin;
  uint8_t _throttlePin;
  bool _reversedDirection;
};

#endif