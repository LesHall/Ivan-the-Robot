// This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
// It won't work with v1.x motor shields! Only for the v2's with built in PWM
// control
// 
// For use with the Adafruit Motor Shield v2 
// ---->	http://www.adafruit.com/products/1438
// 
// Connect the bipolar stepper to M1/M2
// Connect the claw's hobby 9G microservo to M3 and the power input port
// Connect the left hobby 9G microservo to SERVO1
// Connect the right hobby 9G microservo to SERVO2

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
// And connect a DC motor to port M3  this will drive the claw servo
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);

// declare the Arduino Servo library class instances
Servo servo1;  // left servo
Servo servo2;  // right servo
//Servo servo3;  // claw servo



// global variables
float leftPos1 = 90;
float leftPos2 = 180;
float leftPosPrev = 63;
float leftPos = 63;
float rightPos1 = 90;
float rightPos2 = 180;
float rightPosPrev = 63;
float rightPos = 63;
float clawPos1 = 0;
float clawPos2 = 180;
float clawPosPrev = 63;
float clawPos = 63;
float tau = 0.8;
int dly = 10;
float diff = 0.1;
char buff[20];
boolean stringComplete = false;  // true to indicate nwe data is available 


void setup() {
  
  Serial.begin(57600);  // set up Serial library at 9600 bps
 
  AFMS.begin(1000.0/3.0);  // create with 333Hz frequency (3ms period)

 // global variables
  leftPos1 = 90;
  leftPos2 = 180;
  leftPosPrev = 63;
  leftPos = 63;
  rightPos1 = 90;
  rightPos2 = 180;
  rightPosPrev = 63;
  rightPos = 63;
  clawPos1 = 60;
  clawPos2 = 180;
  clawPosPrev = 63;
  clawPos = 63;
  tau = 0.8;
  dly = 10;
  diff = 0.1;
  for (int i = 0; i< 20; i++)
    buff[i] = 0;
  stringComplete = false;

  // Attach servos to pins
  servo1.attach(10);
  servo2.attach(9);
  //servo3.attach(3);
   
  // initialize the stepper
  myStepper->setSpeed(20);  // in units of rpm   
  myStepper->step(0, FORWARD, MICROSTEP);  // move clockwise

  // initialize left and right motor positions
  //servo1.write(constrain(map(leftPos, 0, 127, leftPos1, leftPos2), leftPos1, leftPos2));
  //servo2.write(constrain(map(rightPos, 0, 127, rightPos1, rightPos2), rightPos1, rightPos2));

  // initialize the claw position
  myMotor->setSpeed(clawPos);
  myMotor->run(RELEASE);
  myMotor->run(FORWARD);
}



void loop() {

  // see if there's incoming serial data
  if (stringComplete) {

    //for (int i = 0; i<5; i++) 
    //  buff[i] = Serial.read();
      
    // move stepper one step in direction of commmand
    if ( (int(buff[2]) == 0) && (int(buff[3]) != 0) ) { // buttons up and ready to move
      myStepper->step(3, FORWARD, MICROSTEP);  // move clockwise
      //delay(dly);  // delay to allow commands to take effect
    }
    if ( (int(buff[2]) != 0) && (int(buff[3]) == 0) ) { // buttons up and ready to move
      myStepper->step(3, BACKWARD, MICROSTEP);  // move counter-clockwise
      //delay(dly);  // delay to allow commands to take effect
    }
    
    // update left motor angle
    leftPosPrev = leftPos;
    leftPos = int(buff[0]);
    if (abs(leftPos - leftPosPrev) >= diff) { // more than 'diff' distance apart
      if ( !servo1.attached() ) {  // servo not attached
        servo1.attach(10);  // attach servo
      }
      servo1.write(constrain(map(leftPos, 0, 127, leftPos1, leftPos2), leftPos1, leftPos2));  // move servo
    } else if (servo1.attached()) {  // servo attached
      servo1.detach();  // detach servo
    }
  
    // update right motor angle
    rightPosPrev = rightPos;
    rightPos = int(buff[1]);
    if (abs(rightPos - rightPosPrev) >= diff) { // more than 'diff' distance apart
      if ( !servo2.attached() ) {  // servo not attached
        servo2.attach(9);  // attach servo
      } 
      servo2.write(constrain(map(rightPos, 0, 127, rightPos1, rightPos2), rightPos1, rightPos2));  // move servo
    } else if (servo2.attached()) {  // servo attached
      servo2.detach();  // detach servo
    }
    
    // adjust grip of claw to be open or closed when left mouse button is closed, y axos
    clawPos = int(buff[2]) == 0 ? clawPos1 : clawPos2;
    myMotor->setSpeed(clawPos);
    myMotor->run(RELEASE);
    myMotor->run(FORWARD);

      
    delay(dly);
    stringComplete = false;

    // the following line is from adafruit_support_bill, an amazingly helpful person
    Serial.write('A');  // any old character will do for an 'ack'  
  }
}



// from an example on arduino.cc
// modified by Les Hall circa / prior to Wed Jul 5 2017
void serialEvent() {
  
  while (Serial.available() > 0) {

    // read in the data bytes
    Serial.readBytesUntil(char(128), buff, 20);
    Serial.flush();
    
    stringComplete = true;
  }
}


