#include "DaulHBridge.h"


#define SPEEDPIN1   10 //9 // H-bridge enable pin for speed control motor1
#define MOTOR1APIN  0 // H-bridge leg 1
#define MOTOR2APIN  1 // H-bridge leg 2

#define SPEEDPIN2   12 // 10 // H-bridge enable pin for speed control motor2
#define MOTOR3APIN  3 // H-bridge leg 3
#define MOTOR4APIN  2 // H-bridge leg 2 

DaulHBridge::DaulHBridge()
{
	_pin = 11;
	
	
	pinMode(_pin,OUTPUT);
	
	 _pwmL=SPEEDPIN1;
	 pinMode(_pwmL,OUTPUT);
	 _fwdL=MOTOR1APIN;
	 pinMode(_fwdL,OUTPUT);
	 _revL=MOTOR2APIN;
	 pinMode(_revL,OUTPUT);
	 
	 _pwmR=SPEEDPIN2;
	 pinMode(_pwmR,OUTPUT);
	 _fwdR=MOTOR3APIN;
	 pinMode(_fwdR,OUTPUT);
	 _revR=MOTOR4APIN;

	pinMode(_revR,OUTPUT);
	
	digitalWrite(_pwmL, LOW);
	digitalWrite(_fwdL, LOW);
	digitalWrite(_revL, LOW);
	
	digitalWrite(_pwmR, LOW);
	digitalWrite(_fwdR, LOW);
	digitalWrite(_revR, LOW);
	
	
	
}
void DaulHBridge::init()
{
	  pinMode(_pin,OUTPUT);
	
	 _pwmL=SPEEDPIN1;
	 pinMode(_pwmL,OUTPUT);
	 _fwdL=MOTOR1APIN;
	 pinMode(_fwdL,OUTPUT);
	 _revL=MOTOR2APIN;
	 pinMode(_revL,OUTPUT);
	 
	 _pwmR=SPEEDPIN2;
	 pinMode(_pwmR,OUTPUT);
	 _fwdR=MOTOR3APIN;
	 pinMode(_fwdR,OUTPUT);
	 _revR=MOTOR4APIN;

	pinMode(_revR,OUTPUT);
	
	digitalWrite(_pwmL, LOW);
	digitalWrite(_fwdL, LOW);
	digitalWrite(_revL, LOW);
	
	digitalWrite(_pwmR, LOW);
	digitalWrite(_fwdR, LOW);
	digitalWrite(_revR, LOW);

}
void DaulHBridge::on()
{
	digitalWrite(_pin, HIGH);
	
}
void DaulHBridge::off()
{
	digitalWrite(_pin, LOW);
	
}
// set speed for left motor; speed is a number between -400 and 400
void DaulHBridge::setLeftSpeed(int16_t speed)
{
	
	bool reverse = 0;

    if (speed < 0)
    {
        speed = -speed; // Make speed a positive quantity.
        reverse = 1;    // Preserve the direction.
    }
    if (speed > 400)    // Max PWM duty cycle.
    {
        speed = 400;
    }
	
	
	
	if (reverse == 1)
	{
		digitalWrite(_fwdL, HIGH);
		digitalWrite(_revL, LOW);
		
	}else
	{
		digitalWrite(_fwdL, LOW);
		digitalWrite(_revL, HIGH);
	}
	
	analogWrite(_pwmL,speed);
	
	
}
// set speed for right motor; speed is a number between -400 and 400
void DaulHBridge::setRightSpeed(int16_t speed)
{
		bool reverse = 0;

    if (speed < 0)
    {
        speed = -speed; // Make speed a positive quantity.
        reverse = 1;    // Preserve the direction.
    }
    if (speed > 400)    // Max PWM duty cycle.
    {
        speed = 400;
    }
	
	
	
	if (reverse == 1)
	{
		digitalWrite(_fwdR, HIGH);
		digitalWrite(_revR, LOW);
		
	}else
	{
		digitalWrite(_fwdR, LOW);
		digitalWrite(_revR, HIGH);
	}
	
	analogWrite(_pwmR,speed);
}
