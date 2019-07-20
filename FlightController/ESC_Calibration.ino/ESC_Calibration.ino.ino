#include <Wire.h>
#include <Math.h>

/* SERIAL OUTPUT DEBUG */
//#define DEBUG_ESC

/* TEST OUTPUT CONTROL */
const bool DISABLE_MOTORS = false;  // WARNING: if this is set to false then the motors will turn on!

/* PINOUT ASSIGNMENTS */
const int ESC_4x_ENABLE = B11110000;

/* CONSTANTS */
// ESC input range
static const int MIN_ESC_PULSE = 1000;
// Timekeeping
static const int REFRESH_RATE_MICROS = 4000;  // time interval (in micros) to maintain 250 Hz loop
/* GLOBALS */
static long loop_iter = 0;
// RX
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel8; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4, timer8, currentTime;   // tracks time of rising RX pulse
unsigned int rx_roll_pulse, rx_pitch_pulse, rx_throttle_pulse, rx_yaw_pulse, rx_aux_2;   // RX input pulse
// ESC
unsigned int esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse;  // time duration of ESC pulses (in microsec)
unsigned int loop_timer;  // loop timer to maintain 250Hz refresh rate
/* TIMEKEEPING */
static float time_elapsed = 0.0;      // flight time elapsed
unsigned long previous_time = 0;      // last time of loop start
unsigned long current_time;           // current time of loop start
static float delta_t = 0.0;           // time between loops in sec

byte esc_setting;

void setup() {
  DDRD |= ESC_4x_ENABLE;  // set digital pins 4-7 as outputs for ESC signal outputs
  
  // enable pin change interrupts for RX (by default these pins are enabled as inputs)
  PCICR |= (1 << PCIE0);    // enable PCIEO, any change on PCINT[7:0] will cause interrupt
  PCMSK0 |= (1 << PCINT0);  // enable interrupt on arduino pin 8
  PCMSK0 |= (1 << PCINT1);  // enable interrupt on arduino pin 9
  PCMSK0 |= (1 << PCINT2);  // enable interrupt on arduino pin 10
  PCMSK0 |= (1 << PCINT3);  // enable interrupt on arduino pin 11
  PCMSK0 |= (1 << PCINT4);  // enable interrupt on arduino pin 12

  Serial.begin(115200);
  Serial.println("(Any) Inhibit all ESCs");
  Serial.println("(1) Enable ESC 1");
  Serial.println("(2) Enable ESC 2");
  Serial.println("(3) Enable ESC 3");
  Serial.println("(4) Enable ESC 4");
  Serial.println("(5) Enable all ESCs");
  esc_setting = 48; // decimal 0
}

void loop() {
  // HALT
  while(loop_iter > 4000);
  
  /* MAINTAIN 250HZ LOOP FREQUENCY*/
  while(micros() <= loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros(); 

  /* TIMEKEEPING */
  current_time = millis();
  if(loop_iter > 0) {  // if not first loop iteration
    delta_t = (current_time - previous_time) * 1E-3;   // calculate time passed in seconds
  } 
  
  time_elapsed += delta_t;
  previous_time = current_time;

  if (Serial.available() > 0) {
    esc_setting = Serial.read();  
  }

  /* SEND ESC SIGNALS */
  // set esc input pulse to the throttle pulse from RX
  esc_1_pulse = MIN_ESC_PULSE;
  esc_2_pulse = MIN_ESC_PULSE;
  esc_3_pulse = MIN_ESC_PULSE;
  esc_4_pulse = MIN_ESC_PULSE;
  
  if (esc_setting == 49) { // decimal 1
    esc_1_pulse = rx_throttle_pulse;
  } else if (esc_setting == 50) { // decimal 2
    esc_2_pulse = rx_throttle_pulse;
  }  else if (esc_setting == 51) { // decimal 3
    esc_3_pulse = rx_throttle_pulse;    
  } else if (esc_setting == 52) { // decimal 4
    esc_4_pulse = rx_throttle_pulse;
  } else if (esc_setting == 53) { // decimal 5
    esc_1_pulse = rx_throttle_pulse;
    esc_2_pulse = rx_throttle_pulse;
    esc_3_pulse = rx_throttle_pulse;    
    esc_4_pulse = rx_throttle_pulse;
  }

  send_esc_pulse();
 
  #ifdef DEBUG_ESC
    Serial.println(rx_throttle_pulse);  
  #endif

  loop_iter++;
}

/*  END LOOP  */

// sends pulse to esc with the values of esc_1_pulse, esc_2_pulse, esc_3_pulse, and esc_4_pulse
void send_esc_pulse() {  
  unsigned long offset_timer = micros();  // remember the current time
  PORTD |= ESC_4x_ENABLE;   // set digital pins 4-7 to HIGH
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
    rx_roll_pulse = currentTime - timer1;  // total time of pulse being high
  }

  // handle channel 2
  if((lastChannel2 == 0) && (PINB & B00000010)) {   // pin 9 enabled
    lastChannel2 = 1;
    timer2 = currentTime;    // time of rising pulse
  }
  else if((lastChannel2 == 1) && !(PINB & B00000010)) { // pin 9 disabled
    lastChannel2 = 0;
    rx_pitch_pulse = currentTime - timer2;  // total time of pulse being high
  }

   // handle channel 3
  if((lastChannel3 == 0) && (PINB & B00000100)) {   // pin 10 enabled
    lastChannel3 = 1;
    timer3 = currentTime;    // time of rising pulse
  }
  else if((lastChannel3 == 1) && !(PINB & B00000100)) { // pin 10 disabled
    lastChannel3 = 0;
    rx_throttle_pulse = currentTime - timer3;  // total time of pulse being high
  }

  // handle channel 4
  if((lastChannel4 == 0) && (PINB & B00001000)) {   // pin 11 enabled
    lastChannel4 = 1;
    timer4 = currentTime;    // time of rising pulse
  }
  else if((lastChannel4 == 1) && !(PINB & B00001000)) { // pin 11 disabled
    lastChannel4 = 0;
    rx_yaw_pulse = currentTime - timer4;  // total time of pulse being high
  }

  // handle channel 8
  if((lastChannel8 == 0) && (PINB & B00010000)) {   // pin 12 enabled
    lastChannel8 = 1;
    timer8 = currentTime;    // time of rising pulse
  }
  else if((lastChannel8 == 1) && !(PINB & B00010000)) { // pin 12 disabled
    lastChannel8 = 0;
    rx_aux_2 = currentTime - timer8;  // total time of pulse being high
  }
}
