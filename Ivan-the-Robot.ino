/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->	http://www.adafruit.com/products/1438

This sketch creates a fun motor party on your desk *whiirrr*
Connect a unipolar/bipolar stepper to M3/M4
Connect a DC motor to M1
Connect a hobby servo to SERVO1
*/
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M1 and M2)
Adafruit_StepperMotor *myStepper = AFMS.getStepper(200, 1);
// And connect a DC motor to port M3  this will drive the klaw servo
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

// declare the Arduino Servo library class instances
Servo servo1;  // left servo
Servo servo2;  // right servo
//Servo servo3;  // klaw servo

// global variables
float leftPos1 = 90;
float leftPos2 = 180;
float leftPosPrev = 63;
float leftPos = 63;
float rightPos1 = 90;
float rightPos2 = 180;
float rightPosPrev = 63;
float rightPos = 63;
float klawPos1 = 63 - 30;
float klawPos2 = 63 + 30;
float klawPosPrev = 63;
float klawPos = 63;
float tau = 0.8;
int dly = 10;
float diff = 0.1;
char buff[] = {
  0, 0, 0, 0, 0,
  0, 0, 0, 0, 0};
boolean stringComplete = false;  // whether the string is complete



void setup() {
  
  Serial.begin(9600);  // set up Serial library at 9600 bps
 
  AFMS.begin(1000.0/3.0);  // create with 333Hz frequency (3ms period)
  
  // Attach servos to pins
  servo1.attach(10);
  servo2.attach(9);
  //servo3.attach(3);
   
  // initialize the stepper
  myStepper->setSpeed(20);  // in units of rpm   
  myStepper->step(0, FORWARD, MICROSTEP);  // move clockwise

  // initialize left and right motor positions
  servo1.write(constrain(map(leftPos, 0, 127, leftPos1, leftPos2), leftPos1, leftPos2));
  servo2.write(constrain(map(rightPos, 0, 127, rightPos1, rightPos2), rightPos1, rightPos2));

  // initialize the klaw position
  myMotor->setSpeed(klawPos);
  myMotor->run(RELEASE);
  myMotor->run(FORWARD);
  //servo3.write(constrain(map(klawPos, 0, 127, klawPos1, klawPos2), klawPos1, klawPos2));
}



void loop() {

  // see if there's incoming serial data
  if (stringComplete) {

    //for (int i = 0; i<5; i++) 
    //  buff[i] = Serial.read();
      
    // move stepper one step in direction of commmand
    if ( (int(buff[2]) == 0) && (int(buff[3]) != 0) ) { // buttons up and ready to move
      myStepper->step(1, FORWARD, MICROSTEP);  // move clockwise
      delay(dly);  // delay to allow commands to take effect
    }
    if ( (int(buff[2]) != 0) && (int(buff[3]) == 0) ) { // buttons up and ready to move
      myStepper->step(1, BACKWARD, MICROSTEP);  // move counter-clockwise
      delay(dly);  // delay to allow commands to take effect
    }
    
    // update left motor angle
    leftPosPrev = leftPos;
    leftPos = tau*leftPos + (1-tau)*int(buff[0]);
    //if ( (int(buff[2]) == 0) && (int(buff[3]) == 0) ) { // buttons up and ready to move
      if (abs(leftPos - leftPosPrev) >= diff) { // more than 'diff' distance apart
        if ( !servo1.attached() ) {  // servo not attached
          servo1.attach(10);  // attach servo
        }
        servo1.write(constrain(map(leftPos, 1, 127, leftPos1, leftPos2), leftPos1, leftPos2));  // move servo
      } else if (servo1.attached()) {  // servo attached
        servo1.detach();  // detach servo
      }
      delay(dly);  // delay to allow commands to take effect
    //}
    
    // update right motor angle
    rightPosPrev = rightPos;
    rightPos = tau*rightPos + (1-tau)*int(buff[1]);
    //if ( (int(buff[2]) == 0) && (int(buff[3]) == 0) ) {  // buttons up and ready to move
      if (abs(rightPos - rightPosPrev) >= diff) { // more than 'diff' distance apart
        if ( !servo2.attached() ) {  // servo not attached
          servo2.attach(9);  // attach servo
        } 
        servo2.write(constrain(map(rightPos, 1, 127, rightPos1, rightPos2), rightPos1, rightPos2));  // move servo
      } else if (servo2.attached()) {  // servo attached
        servo2.detach();  // detach servo
      }
      delay(dly);  // delay to allow commands to take effect
    //}
    
    // adjust grip of klaw to be open or closed when left mouse button is closed, y axos
    //if ( (int(buff[2]) == 0) && (int(buff[3]) == 0) ) {  // buttons up and ready to move
      //klawPos = int(int(buff[0]) * (1.0 - cos(2*PI * int(buff[1])/127.0) ) );
      klawPos = int(buff[0]);
      myMotor->setSpeed(constrain(map(klawPos, 1, 127, klawPos1, klawPos2), klawPos1, klawPos2));
      delay(15);
    //}
      
    stringComplete = false;

    // the following line is from adafruit_support_bill, an amazingly helpful person
    Serial.write('A');  // any old character will do for an 'ack'  
  }
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  
  while (Serial.available() > 0) {

    // read in the data bytes
    Serial.readBytesUntil(char(128), buff, 10);
    //Serial.flush();
    
    stringComplete = true;
  }
}


