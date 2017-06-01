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
#define logicVolt 3.3
#define motor1 5
#define motor2 6
#define motor3 9
#define motor4 10
RF24 radio(CE, CS);
rx_values_t rxValues;
// set up controller: pass it radio, channel #, and false since it is not the controller

//PID Code
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define OUTPUT_READABLE_YAWPITCHROLL
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
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprSet[3];        // [yaw, pitch, roll]   intended yaw/pitch/roll container and gravity vector
float yprPrevErr[3];    // [yaw, pitch, roll]   previous yaw/pitch/roll error container and gravity vector
float yprTotErr[3] ;     // [yaw, pitch, roll]   total yaw/pitch/roll error container and gravity vector
float yprCorrec[3]; 
float Kp, Ki, Kd;





Controller controller(&radio, channel, false);
void setup() {
  for(int i =0; i<3; i++)
  {yprTotErr[i]=0;
   yprPrevErr[i]=0;
    }
  MPUsetup();
  Serial.begin(115200);
  // initalize the radio
  controller.init();
  pinMode(motor1, OUTPUT);
  analogWrite(motor1, LOW);
  pinMode(motor2, OUTPUT); 
  analogWrite(motor2, LOW);
  pinMode(motor3, OUTPUT);
  analogWrite(motor3, LOW);
  pinMode(motor4, OUTPUT);
  analogWrite(motor4, LOW);
}

void loop() {
  MPUloop();
  if (!controller.isFunctioning()) {
    Serial.println("EMERGENCY!! TURN OFF ALL MOTORS AND STOP RUNING CODE");
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);
    return;
  }
  //only print values if new values have been received
  //controler.receive will return however many values were in the buffer
  if (controller.receive(&rxValues))
  {
    analogWrite(led, rxValues.throttle);
    yprSet[0]=rxValues.yaw;
    yprSet[1]=rxValues.pitch;
    yprSet[2]=rxValues.roll;
    Kp=rxValues.P;
    Kd=rxValues.D;
    Ki=rxValues.I; 
//    Serial.print(" :\t"); Serial.print(rxValues.throttle);
//    Serial.print("\t"); Serial.print(rxValues.yaw);
//    Serial.print("\t"); Serial.print(rxValues.pitch);
//    Serial.print("\t"); Serial.print(rxValues.roll);
//    Serial.print("\t"); Serial.print(rxValues.flip);
//    Serial.print("\t"); Serial.print(rxValues.highspeed);
//    Serial.print("\t"); Serial.print(rxValues.P);
//    Serial.print("\t"); Serial.print(rxValues.I);
//    Serial.print("\t"); Serial.println(rxValues.D);

    if(rxValues.yaw > 245 && rxValues.pitch > 245){ 
      analogWrite(motor1, 10);
      analogWrite(motor2, 10);
      analogWrite(motor3, 10);
      analogWrite(motor4, 10);
      delay(10); 
    }
    analogWrite(motor1, 0);
    analogWrite(motor2, 0);
    analogWrite(motor3, 0);
    analogWrite(motor4, 0);
  }
  updateBattery();
  // send values for led back to the controller
  controller.send(&rxValues);
  
}
void updateBattery() {
  int batRead = analogRead(contBattPin);
  //calc bat voltage (mV)
  unsigned long batVolt = (batRead * logicVolt * (R3 + R4)) / (R4) * 1000 / 1023;
  if (batVolt < 7400) {
    // tell the controller to turn on the led
    rxValues.auxLED = true;
  }
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void pidController(){
  float yprErr[3];
  float yprDer[3];
  
  for(int i=0; i<3; i++){
    yprErr[i]=ypr[i]* 180/M_PI - yprSet[i]; // P 
    yprDer[i]=yprErr[i]-yprPrevErr[i]; // D 
    yprTotErr[i]+=yprErr[i]; // I
  
    yprCorrec[i] = Kp * yprErr[i] + Ki * yprTotErr[i] + Kd * yprDer[i];
    yprPrevErr[i] = yprErr[i];
  }
}

void readMPU(){
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void MPUsetup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
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

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(-955);
    mpu.setZGyroOffset(5473);
    mpu.setXAccelOffset(144);
    mpu.setYAccelOffset(5);
    mpu.setZAccelOffset(26); // 1688 factory default for my test chip

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
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void MPUloop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
      
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        

        readMPU();
        pidController();
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            Serial.print("ypr\t");
            Serial.print(yprCorrec[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(yprCorrec[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(yprCorrec[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
