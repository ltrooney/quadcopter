/* 
 * This program will have the user perform a series of tests
 * in order to obtain critical values needed for the flight controller
 * code. These values will be stored in the Arduino's EEPROM for use in
 * the main flight controller code.
 */

// const int THROTTLE_MAX
// const int THROTTLE_MIN
// const int ROLL_MAX
// const int ROLL_MIN
// const int PITCH_MAX
// const int PITCH_MIN
// const int YAW_MAX
// const int YAW_MIN

// CH1: roll / aileron
// CH2: pitch / elevator
// CH3: throttle
// CH4: yaw / rudder


// ISR global variables
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4;   // tracks time of rising RX pulse
int CH1, CH2, CH3, CH4;   // RX input pulse

void setup() {
  // enable pin change interrupts
  PCICR |= (1 << PCIE0);    // enable PCIEO, any change on PCINT[7:0] will cause interrupt
  PCMSK0 |= (1 << PCINT0);  // enable interrupt on arduino pin 8
  PCMSK0 |= (1 << PCINT1);  // enable interrupt on arduino pin 9
  PCMSK0 |= (1 << PCINT2);  // enable interrupt on arduino pin 10
  PCMSK0 |= (1 << PCINT3);  // enable interrupt on arduino pin 11
  Serial.begin(9600);
}

void loop() {
  // RX test:
  // 1. Place all sticks in the center position within 5 seconds
  // 2. Place throttle in minimum position within 5 seconds
  // 3. Place throttle in maximum position within 5 seconds
  // 4. Move roll to the left position, then to the right position within 10 seconds
  // 5. Move pitch to the bottom position, then to the top position within 10 seconds
  // 6. Move yaw to the left position, then to the right position within 10 seconds
  delay(25);
}

// see ATmega manual section 14.4 for PINB specification
// PINB0 = B00000001 (arduino input pin 8)
// PINB1 = B00000010 (arduino input pin 9)
// PINB2 = B00000100 (arduino input pin 10)
// PINB3 = B00001000 (arduino input pin 11)

// see ATmega manual section 12.4 for interrupt vector map

/* INTERRUPT SERVICE ROUTINE */
ISR(PCINT0_vect) {
  // handle channel 1
  if((lastChannel1 == 0) && (PINB & B00000001)) {   // pin 8 enabled
    lastChannel1 = 1;
    timer1 = micros();    // time of rising pulse
  }
  else if((lastChannel1 == 1) && !(PINB & B00000001)) { // pin 8 disabled
    lastChannel1 = 0;
    CH1 = micros() - timer1;  // total time of pulse being high
  }

  // handle channel 2
  if((lastChannel2 == 0) && (PINB & B00000010)) {   // pin 9 enabled
    lastChannel2 = 1;
    timer2 = micros();    // time of rising pulse
  }
  else if((lastChannel2 == 1) && !(PINB & B00000010)) { // pin 9 disabled
    lastChannel2 = 0;
    CH2 = micros() - timer2;  // total time of pulse being high
  }

   // handle channel 3
  if((lastChannel3 == 0) && (PINB & B00000100)) {   // pin 10 enabled
    lastChannel3 = 1;
    timer3 = micros();    // time of rising pulse
  }
  else if((lastChannel3 == 1) && !(PINB & B00000100)) { // pin 10 disabled
    lastChannel3 = 0;
    CH3 = micros() - timer3;  // total time of pulse being high
  }

  // handle channel 4
  if((lastChannel4 == 0) && (PINB & B00001000)) {   // pin 11 enabled
    lastChannel4 = 1;
    timer4 = micros();    // time of rising pulse
  }
  else if((lastChannel4 == 1) && !(PINB & B00001000)) { // pin 11 disabled
    lastChannel4 = 0;
    CH4 = micros() - timer4;  // total time of pulse being high
  }
}

void printReg(const int reg) {
  Serial.println(reg, BIN);
}

void printChannels() {
  Serial.print("CH1: ");
  Serial.print(CH1);
  Serial.print(", ");
  Serial.print("CH2: ");
  Serial.print(CH2);
  Serial.print(", ");
  Serial.print("CH3: ");
  Serial.print(CH3);
  Serial.print(", ");
  Serial.print("CH4: ");
  Serial.println(CH4);
}

void plotChannels() {
  Serial.print(CH1);
  Serial.print("\t");
  Serial.print(CH2);
  Serial.print("\t");
  Serial.print(CH3);
  Serial.print("\t");
  Serial.println(CH4);
}
