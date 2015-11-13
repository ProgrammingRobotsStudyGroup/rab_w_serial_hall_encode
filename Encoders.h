// from http://forum.arduino.cc/index.php?topic=160101.0

#pragma once

#include <Arduino.h>

class Encoders
{
  static void isr0 ();
  static void isr1 ();
  
  const byte whichISR_;
  static Encoders * instance0_;
  static Encoders * instance1_;
  
  void handleInterrupt (int, int);
  
  volatile int counter_;
  volatile int pinState;
  
  public:
    Encoders (const byte which);
    void begin (int pinNumber);

    int readEncoder()   { return counter_; }
    void resetEncoder() { counter_ = 0; }
    
    int getPinState() { return pinState; }
};  // end of class Encoders
