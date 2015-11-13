#ifndef DaulHBridge_H
#define DaulHBridge_H
#include <Arduino.h> 
#include "commands.h"

class DaulHBridge {
    public:
	DaulHBridge();
	void on();
	void init();
	void off();
	
	/** \brief Sets the speed for the left motor.
     *
     * \param speed A number from -400 to 400 representing the speed and
     * direction of the left motor.  Values of -400 or less result in full speed
     * reverse, and values of 400 or more result in full speed forward. */
     void setLeftSpeed(int16_t speed);
	 void setRightSpeed(int16_t speed);
	private:
	int _pin;
	
	int _pwmL;
	int _fwdL;
	int _revL;
	int _pwmR;
	int _fwdR;
	int _revR;
	
};
#endif
