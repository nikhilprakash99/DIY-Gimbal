// Libraries for the MPU6050 IMU
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// Pin Declarations
#define INTERRUPT_PIN 2
#define LED_PIN 13
bool blinkState = false;

// Program Config , uncomment to Choose  Desired Config
#define OUTPUT_YPR
//#define LOG_Y
//#define LOG_P
//#define LOG_R
//#define PID_AUTOMATIC
//#define PID_MANUAL   // Option for tuining Mode

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

//Gloabal Variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

//Interupt Routines
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// Initial Setup
void setup()
{
  Serial.begin(9600); // Setting Up serial connection for debugging/logging/tuning

  // enable Arduino interrupt detection
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  IMUsetup();  
}

void loop()
{
  if (!dmpReady) return; // if initialization fails do nothing

  // if Interrupt Signal Comes or the Buffer isnt empty , exit loop and process IMU/DMP Data
  // Else Stay within loop and run Default routines like PID,Logging etc.
  while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // Intended Default Program stuff (PID Control) is present here
    }

  // Processing IMU / DMP Data
  // Note:  this point onwards,Code is only executed when an interrupt signal comes / Buffer has Data left to process
  mpuInterrupt = false;
  fifoCount = mpu.getFIFOCount();
  mpuIntStatus = mpu.getIntStatus();
 
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) { // check for overflow 
        mpu.resetFIFO(); // reset so we can continue cleanly
        fifoCount = mpu.getFIFOCount();
        //Serial.println(F("FIFO overflow!"));
    }
  else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) { // otherwise, check for DMP data ready interrupt
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount(); // wait for correct available data length
        mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        #ifdef OUTPUT_YPR // display Euler angles in degrees
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.println();
        #endif

        #ifdef LOG_P // Display Pitch for logging / Serial Monitor
            Serial.println(ypr[1]);
        #endif

        #ifdef LOG_Y // Display Yaw for logging / Serial Monitor
            Serial.println(ypr[0]);
        #endif

        #ifdef LOG_R // Display Roll for logging / Serial Monitor
            Serial.println(ypr[2]);
        #endif

        digitalWrite(LED_PIN, !blinkState); // blink LED to indicate activity
    }
  
}

// IMU Setup
void IMUsetup()
{
  // Initializing I2C connection 
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
  #endif

  // Initializing the IMU (MPU6050)
  mpu.initialize();
  if(!mpu.testConnection())
    Serial.println("MPU6050 Test connection failed,Check if IMU has been properly connected");

  // User Provided Offsets
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150); 
  mpu.setYAccelOffset(-50); 
  mpu.setZAccelOffset(1060);
  
  // Initializing DMP
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
        mpu.CalibrateAccel(6); // Calibration: generate offsets and calibrate our MPU6050
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
        mpuIntStatus = mpu.getIntStatus();
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}
