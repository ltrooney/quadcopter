import processing.serial.*;

Serial port;
float xPos = 1.0;      // horizontal position of graph
float inByteX = 0;
float inByteY = 0;
float inByteZ = 0;

void setup() {
  size(800, 600);
  println(Serial.list()[2]);
  port = new Serial(this, Serial.list()[2], 9600);
  
  port.bufferUntil('\n');
  
  background(0);
}

void draw() {
  // draw gyro X
  stroke(0, 0, 255);
  line(xPos, height - inByteX + 2, xPos, height - inByteX);
  
  // draw gyro Y
  stroke(0, 255, 0);
  line(xPos, height - inByteY + 2, xPos, height - inByteY);
  
  
  // draw gyro Z
  stroke(0, 0, 255);
  line(xPos, height - inByteZ + 2, xPos, height - inByteZ);
  
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
    String[] gyroVals = split(inString, ' ');
    println(gyroVals);
    inByteX = stringToFloat(gyroVals[0]);
    inByteY = stringToFloat(gyroVals[1]);
    inByteZ = stringToFloat(gyroVals[2]);
  } else {
    println("empty String");
  }
}

float stringToFloat(String inString) {
  return map(float(inString), -10000, 10000, 0, height);
}