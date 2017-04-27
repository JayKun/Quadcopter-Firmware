#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Controller.h"
#include "printf.h"
#include "I2Cdev.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

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
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
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
void mpu_setup(){
   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   // Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    
    devStatus = mpu.dmpInitialize();
    float xGyroOffset = 
    float yGyroOffset =
    float zGyroOffset =
    float zAccelOffset =

  // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(xGyroOffset);
    mpu.setYGyroOffset(yGyroOffset);
    mpu.setZGyroOffset(zGyroOffset);
    mpu.setZAccelOffset(zAccelOffset); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        } 
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
  
}

void read_mpu(){

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif  
  }



void setup() {
  Serial.begin(38400);
  mpu_setup(); 
  // initalize the radio
  controller.init();
  pinMode(led, OUTPUT);
}

void loop() {

  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    land();
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
