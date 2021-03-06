/* Processing code for this example */

 // mouseover serial

 // Demonstrates how to send data to the Arduino I/O board, in order to
 // turn ON a light if the mouse is over a square and turn it off
 // if the mouse is not.

 // created 2003-4
 // based on examples by Casey Reas and Hernando Barragan
 // modified 30 Aug 2011
 // by Tom Igoe
 // This example code is in the public domain.

// modified extensively 
// Les Hall
// Sat Jun3 2017
//

 import processing.serial.*;

 Serial port;

 
int vdiv = 30;
String ports[];
boolean portSelected = false;
int w = 16;
int leftButton = 0;
int rightButton = 0;
boolean motionSelect = false;



 void setup() {
   
   size(510, 510);
   frameRate(motionSelect ? 10 : 20);
   rectMode(RADIUS);
}



void draw() {
  
  // set up this frame
  background(0, 255, 255);
  strokeWeight(5);
  
  // Lissajous patterns for debugging serial port.  
  // Also for demonstration or repetitive task performance.  
  float motionX = width/2 * cos(TWO_PI * frameCount / 250);
  float motionY = height/2 * sin(TWO_PI * frameCount / 250);
  float motionR = width/2 * ( 2 + sin(TWO_PI * frameCount / 1000) ) / 4;

  // convert to polar, then rotate 45 degrees
  float x0 = motionSelect ? motionX : mouseX - width/2;
  float y0 = motionSelect ? motionY : mouseY - height/2;
  float radius = motionSelect ? motionR : sqrt(x0*x0 + y0*y0);
  radius = constrain(radius, 0, width/2);
  float theta = atan2(y0, x0);
  float x1 = width/2 + radius * cos(theta);
  float y1 = height/2 + radius * sin(theta);
  float x = width/2 + radius * cos(theta + PI/4);
  float y = height/2 + radius * sin(theta + PI/4);

  // draw a circle to contain the control motion
  fill(color(255, 0, 255));
  ellipse(width/2, height/2, width, height);

  // draw a marker to follow mouse position
  fill(color(255, 255, 0));
  ellipse(x1, y1, 20, 20);
  
  // display the port list
  if (!portSelected) {

    // List all the available serial ports in the output pane.
    // You will need to choose the port that the Arduino board is
    // connected to from this list. The first port in the list is
    // port #0 and the third port in the list is port #2.
    // if using Processing 2.1 or later, use Serial.printArray()
    ports = Serial.list();
   
    fill(color(255, 255, 255));
    textAlign(CENTER, CENTER);
    textSize(20);
    text("please select a port", width/2, 80);
    textAlign(LEFT, CENTER);
    for (int i = 0; i < ports.length; i++)
      text(str(i) + " " + ports[i], 75, 120 + i*vdiv); 
  }
  
  // add some text to customize it
  fill(color(0, 0, 255));
  textAlign(CENTER, CENTER);
  textSize(40);
  text("Robot Arm", width/2, 40);
  
  // fill byte array with data
  int buff[] = {
    int(constrain(x * 127.0 / width, 1, 127)), 
    int(constrain(y * 127.0 / height, 1, 127)), 
    int(leftButton), 
    int(rightButton), 
    int(128),
  };
  
  // draw lines in the middle indicating X and Y
  stroke(255, 255, 255);
  line(width/2, height/2, width/2 + width/2 * buff[0]/127.0, height/2 + height/2 * buff[1]/127.0);
  
  // write out one set of commands
  if (portSelected) {
    port.clear();
    for (int i = 0; i < buff.length; ++i)
      port.write((buff[i]));
      
    // the following lines from adafruit_support_bill
    while(port.available() != 0) {print("*");} // wait for an ack...
    port.read();  //read the ack  
  }
}



void keyTyped() {  
  
  if (!portSelected) {
    
    // deterimine which port was selected
    int num = int(key - '1') + 1;

    setPort(num);
  }
}



void mouseClicked() {
  
  if (!portSelected) {

    int selection = (mouseY - height/2 + 90) / vdiv;
  
    setPort(selection);
  }
}



void setPort(int num) {
  
  if (!portSelected) {
  
   // Open the port that the Arduino board is connected to (in this case #0)
   // Make sure to open the port at the same speed Arduino is using (9600bps)
   if (num < ports.length) {

     port = new Serial(this, Serial.list()[num], 57600);
     
      if (port != null)
        portSelected = true;
    }
  }
}


void mousePressed() {
  if (mouseButton == LEFT)
    leftButton = width;
  if (mouseButton == RIGHT)
    rightButton = width;
}


void mouseReleased() {
  if (mouseButton == LEFT)
    leftButton = 0;
  if (mouseButton == RIGHT)
    rightButton = 0;
}