#include <Wire.h>
#include <Math.h>

// Pinout assignments
const int ESC_4x_ENABLE = B11110000;
const int LOW_VOLT_LED_ENABLE = B00100000;
const int LOW_VOLT_LED_DISABLE = B11011111;

/* CONSTANTS */
// Quadcopter settings
// RX receiver
static const int MIN_RX_THROTTLE = 1008;
static const int MAX_RX_THROTTLE = 1800;
static const int MIN_RX_ANGLE_PULSE = 1050; // actual: 1004
static const int MAX_RX_ANGLE_PULSE = 1950; // actual: 1964
static const int RX_ACTIVATION_THROTTLE = 1950; // throttle needed to be reached to arm the motors
// ESC input range
static const int MIN_ESC_PULSE = 1000;        // pulse to send to esc to keep from beeping
static const int MAX_ALLOWABLE_PULSE = 1950;  // maximum pulse allowed to be outputted to any ESC
static const int MIN_ALLOWABLE_PULSE = 1020;  // minimum pulse allowed to be outputted to any ESC when throttle is on
// Timekeeping
static const int REFRESH_RATE_MICROS = 4000;  // time interval (in micros) to maintain 250 Hz loop
// IMU register addresses 
static const int MPU6050_addr = 0x68;   // I2C slave address of MPU-6050
static const int PWR_MGMT_1   = 0x6B;   // power register
static const int ACCEL_XOUT_H = 0x3B;   // first accelerometer data register
static const int GYRO_XOUT_H  = 0x43;   // first gyroscope data register
static const int GYRO_CONFIG  = 0x1B;   // gyroscope configuration register
// serial monitor
static const int EN_MOT_1 = 49; // dec 1
static const int EN_MOT_2 = 50; // dec 2
static const int EN_MOT_3 = 51; // dec 3
static const int EN_MOT_4 = 52; // dec 4
static const int EN_ALL_MOT = 53; // dec 5

/* GLOBALS */
// RX
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4, currentTime;   // tracks time of rising RX pulse
unsigned int rx_roll_pulse, rx_pitch_pulse, rx_throttle_pulse, rx_yaw_pulse;   // RX input pulse
// ESC
unsigned int esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse;  // time duration of ESC pulses (in microsec)
unsigned int loop_timer;  // loop timer to maintain 250Hz refresh rate
// IMU raw data
static boolean IMU_found = true;
static int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;   // average gyro offset in each DOF
static int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;      // raw gyro sensor values
static int16_t acc_bias_x, acc_bias_y, acc_bias_z;      // average acc offset in each DOF
static int16_t acc_raw_x, acc_raw_y, acc_raw_z;         // converted values
// IMU calculated data
static boolean startingGyroAnglesSet = false;
static const int sma_size = 15;
static double sma_acc[sma_size];
static double acc_mag = 0;
static double acc_adj_x;
static double acc_adj_y;
static double acc_adj_z;
long i = 0;

// Serial monitor
static int serial_input = 0;

void setup() {
  DDRB |= LOW_VOLT_LED_ENABLE;  // set digital pin 13 as output for low voltage status LED
  DDRD |= ESC_4x_ENABLE;  // set digital pins 4-7 as outputs for ESC signal outputs
  
  // enable pin change interrupts for RX (by default these pins are enabled as inputs)
  PCICR |= (1 << PCIE0);    // enable PCIEO, any change on PCINT[7:0] will cause interrupt
  PCMSK0 |= (1 << PCINT0);  // enable interrupt on arduino pin 8
  PCMSK0 |= (1 << PCINT1);  // enable interrupt on arduino pin 9
  PCMSK0 |= (1 << PCINT2);  // enable interrupt on arduino pin 10
  PCMSK0 |= (1 << PCINT3);  // enable interrupt on arduino pin 11

  Serial.begin(9600);
  Wire.begin();     // initiate Wire library for IMU
  scanI2CDevices(); // identify all connected devices that use I2C  
  while(!IMU_found) {  // halt execution and blink LED if IMU is not located
    PORTB |= LOW_VOLT_LED_ENABLE;  // turn LED on
    delay(500);
    PORTB &= LOW_VOLT_LED_DISABLE; // turn LED off
    delay(500);
  }
  delay(1000);
  initializeIMU();  // power up, set IMU sensitivities, and calculate gyro offset
  
  // pre-loop setup
  // wait until receiver inputs are valid and in lowest position before starting
  // send pulse to ESC to prevent from beeping
  wait_for_motor_arming();  // wait for max throttle input

  while(rx_throttle_pulse > MIN_RX_THROTTLE+4 || rx_throttle_pulse < MIN_ESC_PULSE) {
    if(rx_yaw_pulse > MAX_RX_ANGLE_PULSE || rx_yaw_pulse < MIN_RX_ANGLE_PULSE) {  // disarm motors and wait to be rearmed
      PORTB &= LOW_VOLT_LED_DISABLE;  // turn off led
      wait_for_motor_arming();        // wait for max throttle input
    }
    send_minimum_esc_pulse();
    delay(25);
  }
  // LED blink
  PORTB &= LOW_VOLT_LED_DISABLE;
  delay(125);
  PORTB |= LOW_VOLT_LED_ENABLE;
  delay(125);
  PORTB &= LOW_VOLT_LED_DISABLE;
  delay(125);
  PORTB |= LOW_VOLT_LED_ENABLE;
  delay(125);
  PORTB &= LOW_VOLT_LED_DISABLE; 
}

void loop() {  
  /* MAINTAIN 250HZ LOOP FREQUENCY*/
  while(micros() <= loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros(); 

  /* READ IMU REGISTERS */
  getIMURegisterData();

  /* DETERMINE VIBRATION */
  // zero motion should read 0
  acc_adj_x = (double)(acc_raw_x) - acc_bias_x; 
  acc_adj_y = (double)(acc_raw_y) - acc_bias_y;
  acc_adj_z = (double)(acc_raw_z) - acc_bias_z;

  acc_mag = sqrt((acc_adj_x*acc_adj_x)+(acc_adj_y*acc_adj_y)+(acc_adj_z*acc_adj_z));
  for(int k = 0; k < sma_size-1; k++) { // dequeue oldest value and shift element positions
    sma_acc[k] = sma_acc[k+1];
  }
  sma_acc[sma_size-1] = acc_mag;  // enqueue newest value
    
  /* PRINT TO SERIAL MONITOR */
  if(i++ % 125 == 0) {
    double sum = 0;
    for(int j = 0; j < sma_size; j++) {
      sum += sma_acc[j];
    }
    double avg = sum / 125.0;
    Serial.println(avg);
  }

  // perform SMA filter
//  acc_prev_prev = acc_prev;
//  acc_prev = acc;
//  acc = combined_acc;
//
//  sma_acc = (acc + acc_prev + acc_prev_prev) / 3.0;
//
//  if(serial_input >= EN_MOT_1 && serial_input <= EN_MOT_4) {
//    Serial.print(rx_throttle_pulse); Serial.print("\t");
//    Serial.print(acc_raw_x); Serial.print("\t");
//    Serial.print(acc_raw_y); Serial.print("\t");
//    Serial.print(combined_acc); Serial.print("\t");
//    Serial.println(sma_acc);
//  }
  
  // limit RX inputs
  if(rx_throttle_pulse > MAX_RX_THROTTLE) { rx_throttle_pulse = MAX_RX_THROTTLE; } // maximum throttle === MAX_RX_THROTTLE (~1800us)
  
  esc_1_pulse = MIN_ESC_PULSE;
  esc_2_pulse = MIN_ESC_PULSE;
  esc_3_pulse = MIN_ESC_PULSE;
  esc_4_pulse = MIN_ESC_PULSE;

  /* GET USER INPUT */
  if(Serial.available() > 0) {
    serial_input = Serial.read();
    if(serial_input >= EN_MOT_1 && serial_input <= EN_MOT_4) {
      Serial.print("> testing motor "); Serial.println((serial_input - 48));
    } else if(serial_input == EN_ALL_MOT) {
      Serial.println("> testing all motors");
    } else {
      Serial.println("> STOP all motors");
    }
  }

  // send esc pulse to motor specified by user input
  if(serial_input == EN_MOT_1) {
    esc_1_pulse = rx_throttle_pulse;
  } else if(serial_input == EN_MOT_2) {
    esc_2_pulse = rx_throttle_pulse;
  } else if(serial_input == EN_MOT_3) {
    esc_3_pulse = rx_throttle_pulse;
  } else if(serial_input == EN_MOT_4) {
    esc_4_pulse = rx_throttle_pulse;
  } else if(serial_input == EN_ALL_MOT) {
    esc_1_pulse = rx_throttle_pulse;
    esc_2_pulse = rx_throttle_pulse;
    esc_3_pulse = rx_throttle_pulse;
    esc_4_pulse = rx_throttle_pulse;
  }

  /* ESC CORRECTIONS */
  // if user is not throttling, then cut off motor power
  if(rx_throttle_pulse < MIN_ALLOWABLE_PULSE) {
    esc_1_pulse = MIN_RX_THROTTLE;
    esc_2_pulse = MIN_RX_THROTTLE;
    esc_3_pulse = MIN_RX_THROTTLE;
    esc_4_pulse = MIN_RX_THROTTLE;
  }

  send_esc_pulse();
}

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

void send_minimum_esc_pulse() {
    esc_1_pulse = MIN_ESC_PULSE;
    esc_2_pulse = MIN_ESC_PULSE;
    esc_3_pulse = MIN_ESC_PULSE;
    esc_4_pulse = MIN_ESC_PULSE;
    send_esc_pulse();
}

void wait_for_motor_arming() {
  Serial.println("wait_for_motor_arming()");
  while(rx_throttle_pulse > 1500) {
    send_minimum_esc_pulse();
    delay(25);  // prevent from motors being armed on accident
  }
  
  while(rx_throttle_pulse < RX_ACTIVATION_THROTTLE) {
    send_minimum_esc_pulse();
    delay(25);
  }
  PORTB |= LOW_VOLT_LED_ENABLE; // turn on LED to alert that motors are armed
  delay(250);
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
}

/* HELPERS */

void initializeIMU() {
  // power up the MPU-6050
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(PWR_MGMT_1);   // PWR_MGMT_1 register
  Wire.write(0x09);         // wake-up MPU-6050 when set to 9 and disable temp. sensor
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_addr);
  Wire.write(GYRO_CONFIG);
  
  // set the sensitivity of the gyro:
  byte FS_SEL_1 = B00001000;              // +/- 500 deg/s sensitivity
  Wire.write(FS_SEL_1);

  // set the sensitivity of the accelerometer:
  byte AFS_SEL_0 = B00010000;             // +/- 2g sensitivity
  Wire.write(AFS_SEL_0);
  Wire.endTransmission(true);

  readAndPrintMPURegister(GYRO_CONFIG, 2); // print the contents of GYRO_CONFIG and ACCEL_CONFIG
  calculateSensorBias();  // calculate average error offset from gyro reading
}

void getIMURegisterData() {  
  // initialize data transmission
  Wire.beginTransmission(MPU6050_addr);
  
  Wire.write(ACCEL_XOUT_H);                 // begin reading at this register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true); // request 14 registers for read access
 
  // read accelerometer registers
  acc_raw_x = Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3B (ACCEL_XOUT_L)
  acc_raw_y = Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_raw_z = Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  // skip the 2 temperature registers
  Wire.read()<<8|Wire.read();

  // read gyroscope registers
  gyro_raw_x = Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyro_raw_y = Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyro_raw_z = Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // subtract the calibration offsets
  //acc_raw_x -= acc_bias_x;
  //acc_raw_y -= acc_bias_y;
  //acc_raw_z -= acc_bias_z;
  gyro_raw_x -= gyro_bias_x;
  gyro_raw_y -= gyro_bias_y;     
  gyro_raw_z -= gyro_bias_z;
}

// take the average of the first 2000 gyroscope readings
void calculateSensorBias() {
  Serial.println("Initializing gyroscope calibration...");
  
  long gyro_raw_x = 0, gyro_raw_y = 0, gyro_raw_z = 0;
  long acc_raw_x = 0, acc_raw_y = 0, acc_raw_z = 0;
  const int calibrationIterations = 2000;
  int percent = 0;

  bool led_on = true;

  // calculate offsets
  for(int i = 0; i < calibrationIterations; i++) {
    if(i % (calibrationIterations/10) == 0) {
      percent += 10;
      Serial.print(percent); Serial.println("%");
      if(led_on) {
        PORTB |= LOW_VOLT_LED_ENABLE;
      } else {
        PORTB &= LOW_VOLT_LED_DISABLE;
      }
      led_on = !led_on;
    }
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(ACCEL_XOUT_H);                 // begin reading at this register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_addr, 14, true); // request 14 registers for read access
    
    // read accelerometer registers
    acc_raw_x += Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3B (ACCEL_XOUT_L)
    acc_raw_y += Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acc_raw_z += Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    // skip the 2 temperature registers
    Wire.read()<<8|Wire.read();
    
    // read gyroscope registers
    gyro_raw_x += Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
    gyro_raw_y += Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
    gyro_raw_z += Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
    delay(3);
  }
  
  acc_bias_x = (float)(acc_raw_x) / calibrationIterations;
  acc_bias_y = (float)(acc_raw_y) / calibrationIterations;
  acc_bias_z = (float)(acc_raw_z) / calibrationIterations;
  gyro_bias_x = (float)(gyro_raw_x) / calibrationIterations;
  gyro_bias_y = (float)(gyro_raw_y) / calibrationIterations;
  gyro_bias_z = (float)(gyro_raw_z) / calibrationIterations;
}
void scanI2CDevices() {
  Serial.println("Scanning for I2C devices...");
  byte error, address;
  int numDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0) {    // success
      Serial.print("I2C device found at address 0x");
      if(address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      numDevices++;
    } else if(error == 4) {   // error
      Serial.print("Unknown error at address 0x");  
      if(address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if(numDevices == 0) {
    Serial.println("No I2C devices found.\n");  
    IMU_found = false;
  } else {
    Serial.println("Finished scanning I2C devices.\n");
  }
}

/* PRINT FUNCTIONS */
void readAndPrintMPURegister(const int startingRegisterNumber, const int numRegisters) {
  Serial.print("<Register dump>: "); Serial.print(numRegisters);
  Serial.print(" register(s), starting at 0x"); Serial.println(startingRegisterNumber, HEX);
  
  // queue the registers for reading
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(startingRegisterNumber);    // select register
  Wire.endTransmission();
  
  // read requested registers
  Wire.requestFrom(MPU6050_addr, numRegisters, true); // request read from 2 registers
  for(int i = 0; i < numRegisters; i++) {
    byte REGISTER_VALUE = Wire.read();
    Serial.print("Register 0x"); Serial.print(startingRegisterNumber+i, HEX);
    Serial.print(":\t");  Serial.println(REGISTER_VALUE);
  }
  Serial.println();
}

