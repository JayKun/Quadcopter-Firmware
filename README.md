# Quadcopter-Firmware
Quadcopter Firmware for IEEE project
<h1>Overview</h1>
<h2>Flight Software</h2>

<h3>Idle State</h3>
Idle state has 2 states (a) Active (b) Inactive
Special Commands sequence to exit the idle state.
Implement an idle state during which motors do not spin.

<h3>Radio transmitter</h3>
Download controller.zip --> quadReceiver and transmitter
Unzip in library
Change PID values *t 1.1 1.2 1.3
Kill Command *k
Reset controls *r

There is another library for the MPU which you can get from github.
Read up on MPU chip for fusion reading.

remember to converts the units. Receiver reading (0-255) --> To actual angle reading.  To compare with process variable.Integral Windup. Limit how much the integral can grow. 

Make sure your code is efficient so that each cylce runs in 10ms. Use micors to time your loops. 

