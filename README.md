# rab_w_serial_hall_encode
ROS Arduino Bridge using serial Hall sensors for encoders

This is a version of the ROS_Teensy_Bridge firmware for the Arduino Teensy 2 from Microsat.
It does not require any external libraries such as FastGPIO or other encoders; only the files in the repo are necessary.
Some effort has been made to rip out `#define` preprocessor directives, including `USE_BASE`, which perhaps should be replaced.

The `Encoders` class uses static instances of interrupt service requests (isr) that handle the encoder interrupts.

