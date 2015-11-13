
#include "Encoders.h"

void Encoders::begin (int pinNumber)
  {
  switch (whichISR_)
    {
    case 0: 
      attachInterrupt (pinNumber, isr0, RISING); //CHANGE); //FALLING); 
      instance0_ = this;
      break;
    
    case 1: 
      attachInterrupt (pinNumber, isr1, RISING); //CHANGE); //FALLING); 
      instance1_ = this;
    break;
    }  
  }  // end of Encoders::begin 
  
// constructor
Encoders::Encoders (const byte whichISR) : whichISR_ (whichISR) 
{  
  counter_ = 0;
  pinState = HIGH;
}

// ISR glue routines
void Encoders::isr0 ()
  {
  instance0_->handleInterrupt (0,1);  // get these values from DaulHBridge.cpp
  }  // end of Encoders::isr0

void Encoders::isr1 ()
  {
  instance1_->handleInterrupt (3,2);  
  }  // end of Encoders::isr1
  
// for use by ISR glue routines
Encoders * Encoders::instance0_;
Encoders * Encoders::instance1_;

// class instance to handle an interrupt
void Encoders::handleInterrupt (int pin0, int pin1)
{
  int pinLevel1 = digitalRead(pin1);
  if (pinLevel1 == HIGH) {
    counter_++;
  } else {
    counter_--;
  }
  pinState = !pinState;
}  // end of Encoders::handleInterrupt

