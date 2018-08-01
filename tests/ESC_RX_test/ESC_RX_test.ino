// constants
unsigned int MIN_RX_THROTTLE = 1012;
unsigned int MIN_ESC_THROTTLE = 1000;
unsigned int REFRESH_RATE_MICROS = 4000;  // time interval (in micros) to maintain 250 Hz loop

// ISR global variables
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4, currentTime;   // tracks time of rising RX pulse
unsigned int roll_pulse, pitch_pulse, throttle_pulse, yaw_pulse;   // RX input pulse
unsigned int esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse;  // time duration of ESC pulses (in microsec)

unsigned int loop_timer;  // loop timer to maintain 250Hz refresh rate

void setup() {
  DDRD |= B11110000;  // set digital ports 4-7 as outputs (bits 4-7 of DDRD register)
  
  // enable pin change interrupts for RX (by default these pins are enabled as inputs)
  PCICR |= (1 << PCIE0);    // enable PCIEO, any change on PCINT[7:0] will cause interrupt
  PCMSK0 |= (1 << PCINT0);  // enable interrupt on arduino pin 8
  PCMSK0 |= (1 << PCINT1);  // enable interrupt on arduino pin 9
  PCMSK0 |= (1 << PCINT2);  // enable interrupt on arduino pin 10
  PCMSK0 |= (1 << PCINT3);  // enable interrupt on arduino pin 11

  Serial.begin(9600);
  // pre-loop setup
  // wait until receiver inputs are valid and in lowest position before starting
  // send pulse to ESC to prevent from beeping
//  while(throttle_pulse > MIN_RX_THROTTLE || throttle_pulse < MIN_ESC_THROTTLE) {
//    esc_1_pulse = MIN_ESC_THROTTLE;
//    esc_2_pulse = MIN_ESC_THROTTLE;
//    esc_3_pulse = MIN_ESC_THROTTLE;
//    esc_4_pulse = MIN_ESC_THROTTLE;
//    send_esc_pulse();
//    delay(25);
//  }
}

int loop_iterations = 0;

void loop() {
  // HALT
  //while(loop_iterations > 1250);
  
  /* wait until last loop's start time + 4000 > current loop start time 
   * to maintain a 250Hz loop refresh rate */
  while(micros() <= loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros();
  
  // set esc input pulse to the throttle pulse from RX
  esc_1_pulse = throttle_pulse;
  esc_2_pulse = throttle_pulse;
  esc_3_pulse = throttle_pulse;
  esc_4_pulse = throttle_pulse;

  Serial.print(esc_1_pulse); Serial.print("\t");
  Serial.print(esc_2_pulse); Serial.print("\t");
  Serial.print(esc_3_pulse); Serial.print("\t");
  Serial.println(esc_4_pulse);
  send_esc_pulse();
  
  // TODO: compensate ESC pulse for battery voltage drop & PID corrections
  loop_iterations++;
}

// sends pulse to esc with the values of esc_1_pulse, esc_2_pulse, esc_3_pulse, and esc_4_pulse
void send_esc_pulse() {
  unsigned long offset_timer = micros();  // remember the current time
  PORTD |= B11110000;   // set digital pins 4-7 to HIGH
  /* 
   * HERE THERE IS A 1000us-2000us gap of time, this is where to perform other calculations 
   */
  unsigned long timer_esc_1 = esc_1_pulse + offset_timer; // time that esc 1 pulse should finish
  unsigned long timer_esc_2 = esc_2_pulse + offset_timer; // time that esc 2 pulse should finish
  unsigned long timer_esc_3 = esc_3_pulse + offset_timer; // time that esc 3 pulse should finish
  unsigned long timer_esc_4 = esc_4_pulse + offset_timer; // time that esc 4 pulse should finish
  /* Delay the ESC signal pulse for time equal 
   *  to the pulse width specified by the RX. 
   *  Set the digital outputs to LOW when finished */
  unsigned long curr_time;
  while(PORTD >= 16) { // if PORTD == 16, digital pins 4-7 are set to LOW
    curr_time = micros();
    if(curr_time >= timer_esc_1)  PORTD &= B11101111; // set digital output 4 to LOW
    if(curr_time >= timer_esc_2)  PORTD &= B11011111; // set digital output 5 to LOW
    if(curr_time >= timer_esc_3)  PORTD &= B10111111; // set digital output 6 to LOW
    if(curr_time >= timer_esc_4)  PORTD &= B01111111; // set digital output 7 to LOW
  }
}

/* INTERRUPT SERVICE ROUTINE called when digital inputs 8, 9, 10, or 11 change state */
ISR(PCINT0_vect) { 
  currentTime = micros();
  // handle channel 1
  if((lastChannel1 == 0) && (PINB & B00000001)) {   // pin 8 enabled
    lastChannel1 = 1;
    timer1 = currentTime;    // time of rising pulse
  }
  else if((lastChannel1 == 1) && !(PINB & B00000001)) { // pin 8 disabled
    lastChannel1 = 0;
    roll_pulse = currentTime - timer1;  // total time of pulse being high
  }

  // handle channel 2
  if((lastChannel2 == 0) && (PINB & B00000010)) {   // pin 9 enabled
    lastChannel2 = 1;
    timer2 = currentTime;    // time of rising pulse
  }
  else if((lastChannel2 == 1) && !(PINB & B00000010)) { // pin 9 disabled
    lastChannel2 = 0;
    pitch_pulse = currentTime - timer2;  // total time of pulse being high
  }

   // handle channel 3
  if((lastChannel3 == 0) && (PINB & B00000100)) {   // pin 10 enabled
    lastChannel3 = 1;
    timer3 = currentTime;    // time of rising pulse
  }
  else if((lastChannel3 == 1) && !(PINB & B00000100)) { // pin 10 disabled
    lastChannel3 = 0;
    throttle_pulse = currentTime - timer3;  // total time of pulse being high
  }

  // handle channel 4
  if((lastChannel4 == 0) && (PINB & B00001000)) {   // pin 11 enabled
    lastChannel4 = 1;
    timer4 = currentTime;    // time of rising pulse
  }
  else if((lastChannel4 == 1) && !(PINB & B00001000)) { // pin 11 disabled
    lastChannel4 = 0;
    yaw_pulse = currentTime - timer4;  // total time of pulse being high
  }
}
