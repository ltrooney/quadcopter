#include <Servo.h>

const int PIN_ESC_1 = 4;
const int ESC_1_MIN = 1000;
const int ESC_1_MAX = 2000;

const byte t = 116;
const byte s = 115;

char data;

Servo esc1;

void setup() {
  Serial.begin(9600);
  esc1.attach(PIN_ESC_1, ESC_1_MIN, ESC_1_MAX); // set range of ESC 1 in microseconds
}

void loop() {
  if(Serial.available())
    data = Serial.read();

  if(data == t) {
  // use the write(angle) function where 0 <= angle <= 180
    esc1.write(90);
  } else if(data == s) {
    esc1.write(0);
  }
}
