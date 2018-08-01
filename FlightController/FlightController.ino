#include <Wire.h>
#include <Math.h>

// Pinout assignments
const int PIN_LOW_VOLT_MON = A0;
const int LOW_VOLT_LED_ENABLE = B00100000;
const int LOW_VOLT_LED_DISABLE = B11011111;
const int ESC_4x_ENABLE = B11110000;

/* CONSTANTS */
const int MIN_RX_THROTTLE = 1015;
const int MIN_ESC_THROTTLE = 1000;
const int REFRESH_RATE_MICROS = 4000;  // time interval (in micros) to maintain 250 Hz loop
const float VOLTAGE_MIN = 4; // calculated is 4V (BAT - 11.7V, ARDFC_IN - 10.23V)
// IMU sensitivities
static const float GYRO_SENSITIVITY = 65.5;     // +- 500 deg/s gyroscope sensitivity
static const int16_t ACC_SENSITIVITY = 4096;    // +- 8g accelerometer sensitivity
// IMU register addresses
static const int MPU6050_addr = 0x68;   // I2C slave address of MPU-6050
static const int PWR_MGMT_1   = 0x6B;   // power register
static const int ACCEL_XOUT_H = 0x3B;   // first accelerometer data register
static const int GYRO_XOUT_H  = 0x43;   // first gyroscope data register
static const int GYRO_CONFIG  = 0x1B;   // gyroscope configuration register
// Complementary filter coefficients
static const float GYRO_FUSION_COEFFICIENT = 0.98;
static const float ACC_FUSION_COEFFICIENT = 0.02;
// PID gains
static const float P_GAIN_ROLL = 5;
static const float D_GAIN_ROLL = 0;
static const float I_GAIN_ROLL = 0;
static const float P_GAIN_PITCH = 5;
static const float D_GAIN_PITCH = 0;
static const float I_GAIN_PITCH = 0;
static const float P_GAIN_YAW = 5;
static const float D_GAIN_YAW = 0;
static const float I_GAIN_YAW = 0;

/* GLOBALS */
// RX
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4, currentTime;   // tracks time of rising RX pulse
unsigned int rx_roll_pulse, rx_pitch_pulse, rx_throttle_pulse, rx_yaw_pulse;   // RX input pulse
// ESC
unsigned int esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse;  // time duration of ESC pulses (in microsec)
unsigned int loop_timer;  // loop timer to maintain 250Hz refresh rate
// IMU raw data
static int16_t gyro_bias_x, gyro_bias_y, gyro_bias_z;   // average gyro offset in each DOF
static int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;      // raw gyro sensor values
static int16_t acc_bias_x, acc_bias_y, acc_bias_z;      // average acc offset in each DOF
static int16_t acc_raw_x, acc_raw_y, acc_raw_z;         // converted values
// IMU calculated data
static float gyro_roll, gyro_pitch, gyro_yaw; // angles derived from gyro readings
static float gyro_roll_dot, gyro_pitch_dot, gyro_yaw_dot; // angular velocities derived from gyro readings
static float acc_roll, acc_pitch;   // angles derived from acc readings
static float imu_roll_deg, imu_pitch_deg;           // angles derived from gyro/acc fusion
static boolean startingGyroAnglesSet = false;
// PID calculations
static float error_prior_roll = 0;
static float error_prior_pitch = 0;
static float error_prior_yaw = 0;
static float I_roll = 0;

/* CONVERSIONS */
const float MILLIS_TO_SECONDS = 1E-3;   // convert milliseconds to seconds

/* TIMEKEEPING */
static bool first_loop_iteration = true;
static float time_elapsed = 0.0;      // flight time elapsed
unsigned long previous_time = 0;      // last time of loop start
unsigned long current_time;           // current time of loop start
static float delta_t = 0.0;                 // time between loops in sec

/* TEST OUTPUT CONTROL */
const bool PRINT_LOW_VOLTAGE_WARNING  = false;
const bool PRINT_ESC_SIGNALS  = false;
const bool PRINT_RX_SIGNALS   = false;
const bool PRINT_GYRO_ANGLES  = false;
const bool PRINT_ACC_ANGLES   = false;
const bool PRINT_FUSED_ANGLES = false;
const bool PRINT_ELAPSED_TIME = false;

void setup() {
  DDRB |= LOW_VOLT_LED_ENABLE;  // set digital pin 13 as output for low voltage status LED
  DDRC &= B11111110;      // set analog pin A1 as input for low voltage monitor
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
  initializeIMU();  // power up, set IMU sensitivities, and calculate gyro offset
  
  // pre-loop setup
  // wait until receiver inputs are valid and in lowest position before starting
  // send pulse to ESC to prevent from beeping

  /*
  while(rx_throttle_pulse > MIN_RX_THROTTLE || rx_throttle_pulse < MIN_ESC_THROTTLE) {
    esc_1_pulse = MIN_ESC_THROTTLE;
    esc_2_pulse = MIN_ESC_THROTTLE;
    esc_3_pulse = MIN_ESC_THROTTLE;
    esc_4_pulse = MIN_ESC_THROTTLE;
    send_esc_pulse();
    delay(40);
  }
  */
}

void loop() {
  /* MAINTAIN 250HZ LOOP FREQUENCY*/
  while(micros() <= loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros(); 

  /* LOW VOLTAGE BATTERY MONITOR */
  int sensorValue = analogRead(PIN_LOW_VOLT_MON); // read value of pin A0 (value in range 0-1023)
  float voltage = sensorValue * (5.0 / 1023.0);   // convert sensorValue to voltage between 0V-5V
  if(voltage < VOLTAGE_MIN) {
    if(PRINT_LOW_VOLTAGE_WARNING) {
      Serial.print("WARNING: Low voltage - ");  Serial.println(voltage);
    }
    PORTB |= LOW_VOLT_LED_ENABLE;   // set digital output 13 HIGH
  } else {
    PORTB &= LOW_VOLT_LED_DISABLE; // set digital output 13 LOW
  }

  /* TIMEKEEPING */
  current_time = millis();
  if(!first_loop_iteration) {  // if not first loop iteration
    delta_t = (current_time - previous_time) * MILLIS_TO_SECONDS;   // calculate time passed in seconds
  } 
  first_loop_iteration = false;
  
  time_elapsed += delta_t;
  previous_time = current_time;
  
  if(PRINT_ELAPSED_TIME) printElapsedFlightTime();

  /* READ IMU REGISTERS */
  getIMURegisterData();

  /* GYRO CALCULATIONS */
  // convert from integer to angular velocity in deg/s
  gyro_roll_dot = gyro_raw_y / GYRO_SENSITIVITY;
  gyro_pitch_dot = gyro_raw_x / GYRO_SENSITIVITY;
  gyro_yaw_dot = gyro_raw_z / GYRO_SENSITIVITY;

  // integrate to obtain roll, pitch, and yaw values
  gyro_roll += gyro_roll_dot * delta_t;
  gyro_pitch += gyro_pitch_dot * delta_t;
  gyro_yaw += gyro_yaw_dot * delta_t;

  /* ACCELEROMETER CALCULATIONS */
  const float acc_x_in_g = acc_raw_x/ACC_SENSITIVITY; // convert acc x from int to g
  const float acc_y_in_g = acc_raw_y/ACC_SENSITIVITY; // convert acc y from int to g
  const float acc_z_in_g = acc_raw_z/ACC_SENSITIVITY; // convert acc z from int to g
  const float acc_pitch_radians  = atan2(acc_y_in_g, acc_z_in_g);  // calculate pitch in radians
  const float acc_roll_radians = atan2(-acc_x_in_g, sqrt((acc_y_in_g*acc_y_in_g) + (acc_z_in_g*acc_z_in_g)));  // calculate roll in radians
  acc_roll = acc_roll_radians * RAD_TO_DEG;   // convert acc roll from radians to degrees
  acc_pitch = acc_pitch_radians * RAD_TO_DEG; // conver acc pitch from radians to degrees

  /* GYRO/ACC FUSION CALCULATION */
  if(startingGyroAnglesSet) {
    // use complementary filter to fuse sensor readings
    imu_roll_deg = (GYRO_FUSION_COEFFICIENT*gyro_roll) + (ACC_FUSION_COEFFICIENT*acc_roll); 
    imu_pitch_deg = (GYRO_FUSION_COEFFICIENT*gyro_pitch) + (ACC_FUSION_COEFFICIENT*acc_pitch);   
  } else {
    // accomodate for any starting angular offset due to unlevel surface
    gyro_roll = acc_roll;
    gyro_pitch = acc_pitch;
    startingGyroAnglesSet = true;
  }
  
  if(PRINT_GYRO_ANGLES) plotGyroAngles();
  if(PRINT_ACC_ANGLES) plotAccAngles();
  if(PRINT_FUSED_ANGLES) plotFusedAngles();

  /* PID CALCULATIONS - ROLL */
  rx_throttle_pulse = 1500;
  rx_roll_pulse = 1500; // test roll
  const float rx_roll_deg = (rx_roll_pulse/20) - 75; // convert roll from 1000us -> 2000us to -25 deg -> +25 deg
  const float error_roll = rx_roll_deg - imu_roll_deg;    // error = desired - actual measurement
  const float P_roll = P_GAIN_ROLL * error_roll;
  I_roll += I_GAIN_ROLL * error_roll;
  const float D_roll = D_GAIN_ROLL * (error_roll - error_prior_roll);
  error_prior_roll = error_roll;
  const float roll_pid = (int)(P_roll + I_roll + D_roll);

  /* SEND ESC SIGNALS */
  // set esc input pulse to the throttle pulse from RX
  esc_1_pulse = rx_throttle_pulse;
  esc_2_pulse = rx_throttle_pulse - roll_pid;
  esc_3_pulse = rx_throttle_pulse;
  esc_4_pulse = rx_throttle_pulse + roll_pid;
  send_esc_pulse();

  Serial.print(rx_throttle_pulse);  Serial.print("\t");
  Serial.print(esc_2_pulse);  Serial.print("\t");
  Serial.print(esc_4_pulse);  Serial.println("\t");

  // TODO: compensate ESC pulse for battery voltage drop & PID corrections
}

// sends pulse to esc with the values of esc_1_pulse, esc_2_pulse, esc_3_pulse, and esc_4_pulse
void send_esc_pulse() {
  if(PRINT_ESC_SIGNALS) print_esc_signals();
  if(PRINT_RX_SIGNALS) print_rx_signals();
  
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
}

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
  acc_raw_x -= acc_bias_x;
  acc_raw_y -= acc_bias_y;
  acc_raw_z -= acc_bias_z;
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

  // calculate offsets
  for(int i = 0; i < calibrationIterations; i++) {
    if(i % (calibrationIterations/10) == 0) {
      percent += 10;
      Serial.print(percent); Serial.println("%");
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
  
  Serial.println("gyroscope calibration values collected!");

  acc_bias_x = acc_raw_x / calibrationIterations;
  acc_bias_y = acc_raw_y / calibrationIterations;
  acc_bias_z = acc_raw_z / calibrationIterations;
  gyro_bias_x = gyro_raw_x / calibrationIterations;
  gyro_bias_y = gyro_raw_y / calibrationIterations;
  gyro_bias_z = gyro_raw_z / calibrationIterations;

  // print the averages
  Serial.print("accelerometer biases [x, y, z]:\t");
  plot3Angles(acc_bias_x, acc_bias_y, acc_bias_z); 
  Serial.print("gyroscope biases [x, y, z]:\t");
  plot3Angles(gyro_bias_x, gyro_bias_y, gyro_bias_z);
}

/* HELPERS */
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

  if(numDevices == 0)
    Serial.println("No I2C devices found.\n");  
  else
    Serial.println("Finished scanning I2C devices.\n");
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

void print_esc_signals() {
  Serial.print("1: "); Serial.print(esc_1_pulse);
  Serial.print(", 2: "); Serial.print(esc_2_pulse);
  Serial.print(", 3: "); Serial.print(esc_3_pulse);
  Serial.print(", 4: "); Serial.println(esc_4_pulse);
}

void print_rx_signals() {
  Serial.print("Throttle: "); Serial.print(rx_throttle_pulse);
  Serial.print(", Yaw: "); Serial.print(rx_yaw_pulse);
  Serial.print(", Pitch: "); Serial.print(rx_pitch_pulse);
  Serial.print(", Roll: "); Serial.println(rx_roll_pulse);  
}

void plotAngle(float angle) {
  Serial.println(angle);
}

void plot2Angles(float a1, float a2) {
  Serial.print(a1);
  Serial.print("\t");
  Serial.println(a2);
}

void plot3Angles(float a1, float a2, float a3) {
  Serial.print(a1);
  Serial.print("\t"); Serial.print(a2);
  Serial.print("\t"); Serial.println(a3);
}

void plotGyroAngles() {
  plot3Angles(gyro_roll, gyro_pitch, gyro_yaw);
}

void plotAccAngles() {
  plot2Angles(acc_roll, acc_pitch);
}

void plotFusedAngles() {
  plot2Angles(imu_roll_deg, imu_pitch_deg);
}

void plotAngularVelocities() {
  plot3Angles(gyro_roll_dot, gyro_pitch_dot, gyro_yaw_dot);
}

void printIMUValues() {
  Serial.print("gyro [r,p,y]: ");
  plotGyroAngles();
  Serial.print("acc  [r,p]: ");
  plotAccAngles();
}

void printElapsedFlightTime() {
  Serial.print(time_elapsed);  Serial.println("s");
}

