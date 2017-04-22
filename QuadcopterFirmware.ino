#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Controller.h"
#include "printf.h"

//THIS IS YOUR TEAM NUMBER
#define channel 23
#define PALevel RF24_PA_HIGH
#define CE A0
#define CS A1
// test led (should be pwm)
#define led 3
/********************************
 **** Voltage Divider Consts ****
 ********************************/
#define contBattPin A2
#define R4 1600
#define R3 1000
#define logicVoltage 3.3

// Initialize the radio
RF24 radio(CE, CS);
rx_values_t rxValues;
 
// Initialize the MPU TODO
MPU6050 mpu;

// set up controller: pass it radio, channel #, and false since it is not the controller
Controller controller(&radio, channel, false);

//PID Values // TODO
float Kd;
float Ki;
float Kp; 

// Initialize Variables for readings TODO
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


//Get Adjusted Errors TODO
void pidController(float &yawPrevError, float &rollPrevError, float &pitchPrevError){
  yawError = yawSet - yawRead 
  totalYawError += yawError
  yawDerivative = yawError - yawPrevError
  yawCorrection = Kp * yawError + Ki * totalYawError + Kd * yawDerivative
  yawPrevError = yawError 

  rollError = rollSet - rollRead 
  totalRollError += rollError
  rollDerivative = rollrror - rollPrevError
  rollCorrection = Kp * rollError + Ki * totalRollError + Kd * rollDerivative
  rollPrevError = rollError 

  // same for pitch
  pitchError = pitchSet - pitchRead 
  totalpitchError += pitchError
  pitchDerivative = pitchError - pitchPrevError
  pitchCorrection = Kp * pitchError + Ki * totalpitchError + Kd * pitchDerivative
  pitchPrevError = pitchError 
}

// MPU get basereadings TODO
void mpu_getBaseReadings(){
  float xGyroOffset = 
  float yGyroOffset =
  float zGyroOffset =
  float zAccelOffset =

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(xGyroOffset);
  mpu.setYGyroOffset(yGyroOffset);
  mpu.setZGyroOffset(zGyroOffset);
  mpu.setZAccelOffset(zAccelOffset); // 1688 factory default for my test chip
  
}

void setMotorOutputs(){
  motorCFront=somefactor*pitchError + somefactor*rollError + somefactor*yawError
  setMotorOutput(motorCFront)
}


void setup() {
  Serial.begin(38400);
  
  // initalize the radio
  controller.init();
  pinMode(led, OUTPUT);
}
void loop() {

  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    return;
  }
  //only print values if new values have been received
  //controler.receive will return however many values were in the buffer
  if (controller.receive(&rxValues))
  {
    
  //2. Map joystick positions to -30 and 30 degrees TODO
   uint8_t yawReading = map(rxValues.yaw, -127, 126, -30, 30);
   uint8_t pitchReading = map(rxValues.pitch, -127, 126, -30, 30); 
   uint8_t rollReading = map(rxValues.roll, -127, 126, -30, 30);

  

    
    analogWrite(led, rxValues.throttle);
    Serial.print(" :\t"); Serial.print(rxValues.throttle);
    Serial.print("\t"); Serial.print(rxValues.yaw);
    Serial.print("\t"); Serial.print(rxValues.pitch);
    Serial.print("\t"); Serial.print(rxValues.roll);
    Serial.print("\t"); Serial.print(rxValues.flip);
    Serial.print("\t"); Serial.print(rxValues.highspeed);
    Serial.print("\t"); Serial.print(rxValues.P);
    Serial.print("\t"); Serial.print(rxValues.I);
    Serial.print("\t"); Serial.println(rxValues.D);
  }
  updateBattery();
  
  // send values for led back to the controller 
  controller.send(&rxValues);
}
void updateBattery() {
  int batRead = analogRead(contBattPin);
  //calc bat voltage (mV)
  unsigned long batVolt = (batRead * logicVoltage * (R3 + R4)) / (R4) * 1000 / 1023;
  if (batVolt < 7400) {
    // tell the controller to turn on the led
    rxValues.auxLED = true;
  }
}
