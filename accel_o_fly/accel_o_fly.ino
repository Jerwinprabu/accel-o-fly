// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// this was modified from original sourced @:
// https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/Examples/MPU6050_DMP6
// on 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// todo!

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

copyright removed for time being. 
TODO: make this MPU6050 a library or such and move all the setup stuff out to that
===============================================
*/

// include servo lib
// #include <Servo.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

// include the LCD library
//#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
// reversed with new breadboard 
//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// setup the led matrix
#include "HT1632.h"

#define DATA 4
#define WR   5
#define CS   6
// #define CS2  0

HT1632LEDMatrix matrix = HT1632LEDMatrix(DATA, WR, CS);

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len) - For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */


// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
// #define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

 #define OUTPUT_TO_SERIAL

// #define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
// bool blinkState = false;

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

// interupt for button to level
int intPin = 3;

// define vars for angles to display
int bankAngleDegrees;
int pitchAngleDegrees;
int bankAngleDegreesShort;
int pitchAngleDegreesShort;
int bankAngleLevel;
int pitchAngleLevel;

int matrixBrightness = 1;


// We'll also declare a global variable for the data we're
// sending to the shift register:

byte data = 0;


// setup custom chars for lcd display
            byte pitchUp[8] = {
              0b00100,
              0b01110,
              0b11111,
              0b00100,
              0b00100,
              0b00100,
              0b00100,
              0b00100
            };     

            byte pitchDown[8] = {
              0b00100,
              0b00100,
              0b00100,
              0b00100,
              0b11111,
              0b01110,
              0b01110,
              0b00100,
            };     

    // setup servo
//    Servo myservo;
//    int servoPos = 0; 
    
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

  // button interupt
  volatile int buttonState = LOW;
  volatile unsigned long last_interrupt_time = 0;


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  
    // define button as interrupt
    pinMode(intPin, INPUT_PULLUP);
    attachInterrupt(1,setLevel,CHANGE);


//    // set up the LCD's number of columns and rows: 
//    lcd.begin(16, 2);
//    lcd.clear();
//
//    lcd.setCursor(0,0); // set to top left
//    lcd.print("Bank");
//
//    lcd.setCursor(11,0); // set to top left
//    lcd.print("Pitch");
  
  
  // init LED matrix
    matrix.begin(HT1632_COMMON_16NMOS);  
    matrix.setBrightness(matrixBrightness);

      // box for pitch 
       matrix.drawLine(10,0,10,15,1);      
       matrix.drawLine(13,0,13,15,1);      

  
    // attach servo
//    myservo.attach(5);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(38400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
//    wait for a key to start
//    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
   
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      
// other code here    
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

  //  always get fifo data and save to vars
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetEuler(euler, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            pitchAngleDegreesShort = (ypr[1]*-1 * 180/M_PI)*-100;
            bankAngleDegreesShort = (ypr[2]* -1 * 180/M_PI)*-100;
            pitchAngleDegrees = (pitchAngleDegreesShort - pitchAngleLevel)/100;
            bankAngleDegrees = (bankAngleDegreesShort - bankAngleLevel)/100;

//        #ifdef OUTPUT_READABLE_QUATERNION
//            // display quaternion values in easy matrix form: w x y z
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            Serial.print("quat\t");
//            Serial.print(q.w);
//            Serial.print("\t");
//            Serial.print(q.x);
//            Serial.print("\t");
//            Serial.print(q.y);
//            Serial.print("\t");
//            Serial.println(q.z);
//        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("raw ypr:\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[2] * 180/M_PI);
//            Serial.print("\t leveled: ");
//            Serial.print(pitchAngleLevel);
//            Serial.print("\t");            
//            Serial.print(bankAngleLevel);
//            servoPos = bankAngleDegrees + 90;
//            myservo.write(servoPos);
//            Serial.print("\t");
//            Serial.println(servoPos);
              Serial.println();
        #endif
            
        #ifdef OUTPUT_TO_SERIAL
            // display Euler angles in degrees
            Serial.print("raw ypr:\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[2] * 180/M_PI);
            Serial.print("\t leveled: ");
            Serial.print(pitchAngleLevel/100);
            Serial.print("\t");            
            Serial.print(bankAngleLevel/100);
            Serial.print("\t");
//            Serial.println(servoPos);
        #endif

//        #ifdef OUTPUT_READABLE_REALACCEL
//            // display real acceleration, adjusted to remove gravity
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            Serial.print("areal\t");
//            Serial.print(aaReal.x);
//            Serial.print("\t");
//            Serial.print(aaReal.y);
//            Serial.print("\t");
//            Serial.println(aaReal.z);
//        #endif
//
//        #ifdef OUTPUT_READABLE_WORLDACCEL
//            // display initial world-frame acceleration, adjusted to remove gravity
//            // and rotated based on known orientation from quaternion
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetAccel(&aa, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//            Serial.print("aworld\t");
//            Serial.print(aaWorld.x);
//            Serial.print("\t");
//            Serial.print(aaWorld.y);
//            Serial.print("\t");
//            Serial.println(aaWorld.z);
//        #endif


  // LED matrix
  
      int linePositionLeft;
      linePositionLeft = map(bankAngleDegrees,-30,30,0,15);
      int linePositionRight;
      linePositionRight = map(bankAngleDegrees,-30,30,15,0);
      int linePositionPitch;
      linePositionPitch = map(constrain(pitchAngleDegrees,-35,30),-35,30,0,15);
   
      // matrix.clearScreen();
      // clear manually with a empty rect, as clearsScreen is causing flicker
      matrix.fillRect(0,0,10,16,0);
      matrix.fillRect(14,0,10,16,0);
      
      if ( (bankAngleDegrees > -4) && (bankAngleDegrees < 4) )
        {
          matrix.drawLine(0,8,23,8,1);
        }
      else
        {
          matrix.drawLine(0,linePositionLeft,23,linePositionRight,1);
        }

      // clear out pitch box
      matrix.fillRect(11,0,2,15,0);

      matrix.fillRect(11,linePositionPitch,2,15,1);
      matrix.writeScreen();
      
//        // blink LED to indicate activity
//        blinkState = !blinkState;
//      digitalWrite(LED_PIN, blinkState);
        
// uncomment to use LCD again
        
//   // Print bank angle to the LCD
//        lcd.setCursor(2,1);
//        lcd.print("  ");
//        if (abs(bankAngleDegrees) < 10) 
//          {
//            lcd.setCursor(3,1); 
//          }
//          else         
//          {
//            lcd.setCursor(2,1); 
//          }
//        lcd.print(abs(bankAngleDegrees));
//
//   // Print pitch angle to the LCD
//        lcd.setCursor(13,1);
//        lcd.print("   ");
//        if (pitchAngleDegrees >=0 && pitchAngleDegrees < 10)
//          {
//            lcd.setCursor(15,1); 
//          }
//          else if (pitchAngleDegrees <= 0 && pitchAngleDegrees > -10 )
//          {
//             lcd.setCursor(14,1);
//          }
//          else if (pitchAngleDegrees <= -10)
//          {
//             lcd.setCursor(13,1);
//          }
//          else
//          {
//             lcd.setCursor(14,1);
//          }
//
//        lcd.print(pitchAngleDegrees);
//
//  // show left or right for bank direction
//  
//        if (bankAngleDegrees > 2)
//        {
//          lcd.setCursor(5,1);
//          lcd.print(">");     
//          lcd.setCursor(0,1);
//          lcd.print(" ");     
//        }
//        else if (bankAngleDegrees < -2)
//        {
//          lcd.setCursor(0,1);
//          lcd.print("<");     
//          lcd.setCursor(5,1);
//          lcd.print(" ");     
//        }
//        else
//        {
//          lcd.setCursor(0,1);
//          lcd.print(" ");     
//          lcd.setCursor(5,1);
//          lcd.print(" ");     
//        }
//        
//
//  // show up or down arrow for pitch
//        lcd.createChar(5, pitchUp);        
//        lcd.createChar(6, pitchDown);        
//
//        if (pitchAngleDegrees > 0)
//          {
//             lcd.setCursor(10,1);
//             lcd.write(5);
//          }
//         else 
//         {
//             lcd.setCursor(10,1);
//             lcd.write(6);
//         }
//         
//
//          // show buffer for troubleshooting
////        lcd.setCursor(0,1);
////        lcd.print(fifoCount);
//
//

    }
}

    void setLevel()
      {

        bankAngleLevel = bankAngleDegreesShort;
        pitchAngleLevel = pitchAngleDegreesShort;  
        matrix.clearScreen();
      // box for pitch 
       matrix.drawLine(10,0,10,15,1);      
       matrix.drawLine(13,0,13,15,1);      
        Serial.println("Leveled!\t");              

        
//        // If interrupts come faster than x00ms, assume it's a bounce and ignore
//          if (millis() - last_interrupt_time > 500)
//          {
//                if (matrixBrightness == 1)
//                {
//                  matrixBrightness = 15;
//                  matrix.setBrightness(matrixBrightness);
//                }
//                else
//                {
//                  matrixBrightness = 1;
//                  matrix.setBrightness(matrixBrightness);
//                }
//                Serial.print(millis());
//                Serial.print("\t");
//                Serial.println(matrixBrightness);
//          }
//          
//          last_interrupt_time = millis();
      }
         
         
          


