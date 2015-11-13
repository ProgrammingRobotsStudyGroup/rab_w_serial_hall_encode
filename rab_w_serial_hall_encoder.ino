// rab_w_serial_hall_encoder - ROS Arduino Bridge with serial Hall sensor encoders
// 
// Jay Salmonson
// 9/15/2015

int ledPin = 11;

/* Include definition of serial commands */
#include "commands.h"

// ---- Encoder stuff ----
#include "Encoders.h"

// instances of class
Encoders leftEncoder  (0);
Encoders rightEncoder (1);


// ---- Motor stuff ----

#include "motor_driver.h"

unsigned char moving = 0; // is the base in motion?  (from diff_control.h)
/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

// ---- Serial stuff ----

#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

int arg = 0;
int index = 0;

// ---- PID parameters and functions ---

#include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;


// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0; //NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
void runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
    
  case READ_ENCODERS:
    Serial.print(leftEncoder.readEncoder());
    Serial.print(" ");
    Serial.println(rightEncoder.readEncoder());
    break;
   case RESET_ENCODERS:
    leftEncoder.resetEncoder();
    rightEncoder.resetEncoder();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeeds(0, 0);
      moving = 0;
    }
    else {
      // simple by-pass of PID:
      // setMotorSpeeds(arg1, arg2);
      moving = 1;
    }
    leftPID.TargetTicksPerFrame = arg1;
    rightPID.TargetTicksPerFrame = arg2;
    Serial.print(leftPID.TargetTicksPerFrame);
    Serial.print(" ");
    Serial.println(rightPID.TargetTicksPerFrame);

    Serial.println("OK"); 
    break;
  case UPDATE_PID:
  
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    //Serial.println("PID not implemented.");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}


void setup()
{
  Serial.begin(BAUDRATE);
  
  pinMode(ledPin, OUTPUT);

  pinMode(14, INPUT); // test reading voltage input
  
  leftEncoder.begin(5); 
  rightEncoder.begin(7); 
  
  initMotorController();
  resetPID();
}  // end of setup

void loop() { 
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
      
      /*
      // Trying to read pins:
      int SensorValue0  = digitalRead(0);
      int SensorValue1  = digitalRead(1);
      int PWMValue10    = analogRead(10);
      int SensorValue2  = digitalRead(2);
      int SensorValue3  = digitalRead(3);  
      int PWMValue12    = analogRead(12);
      Serial.print("          ");
      Serial.print(SensorValue0);
      Serial.print(" -> ");
      Serial.print(SensorValue1);
      Serial.print(" speed ");
      Serial.print(PWMValue10);
      Serial.print(" and ");
      Serial.print(SensorValue2);
      Serial.print(" -> ");
      Serial.print(SensorValue3);
      Serial.print(" speed ");
      Serial.print(PWMValue12);
      Serial.print("  pin14: ");
      // for some reason reading pin 10 with pin 14 just gives a binary result.  Is that the PWM thing; just catching it while its one or off?
      int SensorValue14 = analogRead(14);
      Serial.println(SensorValue14);
      
      if ((SensorValue0 ^ SensorValue1) == 0 || (SensorValue2 ^ SensorValue3) == 0) {
        Serial.println("Direction Error!");
      }
      */

    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  // If we are using base control, run a PID calculation at the appropriate intervals
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
    setMotorSpeeds(0, 0);
    moving = 0;
  }

}
