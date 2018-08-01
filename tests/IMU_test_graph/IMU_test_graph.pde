import processing.serial.*;

Serial port;
float xPos = 1.0;      // horizontal position of graph
float inByte = 0;

void setup() {
  size(800, 600);
  println(Serial.list()[2]);
  port = new Serial(this, Serial.list()[2], 9600);
  
  port.bufferUntil('\n');
  
  background(0);
}

void draw() {
  // draw the line
  stroke(127, 34, 255);
  line(xPos, height, xPos, height - inByte);
  
  // at edge of screen go back to beginning
  if(xPos >= width) {
    xPos = 0;
    background(0);
  } else {
    xPos+=0.5;
  }
}

void serialEvent(Serial port) {
  String inString = port.readStringUntil('\n');
    
  if(inString != null) {
    inString = trim(inString);
    inByte = float(inString);    // convert to float
    println(inString);
    inByte = map(inByte, -10000, 10000, 0, height);
  } else {
    println("empty String");
  }
}