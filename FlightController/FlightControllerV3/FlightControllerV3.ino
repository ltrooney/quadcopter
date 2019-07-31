#include <PIDController.h>
#include <LowPassFilter.h>
#include <Wire.h>
#include <Math.h>

/* SERIAL OUTPUT DEBUG */
#define DEBUG_BOOTUP
//#define DEBUG_ACC_ANGLES
//#define DEBUG_GYRO_ANGLES
//#define DEBUG_ANGLES
//#define DEBUG_RAW_RX_PULSE
//#define DEBUG_FILTERED_RX_PULSE
//#define DEBUG_RX_DEG
//#define DEBUG_PID
//#define DEBUG_ESC

#ifdef DEBUG_BOOTUP
  #define PRINT_BOOTUP_SEQ(x) (Serial.print(x))
  #define PRINTLN_BOOTUP_SEQ(x) (Serial.println)(x)
  #define PRINT_BOOTUP_SEQ_HEX(x) (Serial.print(x, HEX))
  #define PRINTLN_BOOTUP_SEQ_HEX(x) (Serial.println(x, HEX))
#else
  #define PRINT_BOOTUP_SEQ(x)
  #define PRINTLN_BOOTUP_SEQ(x)
  #define PRINT_BOOTUP_SEQ_HEX(x)
  #define PRINTLN_BOOTUP_SEQ_HEX(x)
#endif

const int PRINT_RATE = 5; // prints every PRINT_RATE loop iterations

/* TEST OUTPUT CONTROL */
const bool DISABLE_MOTORS = false;  // WARNING: if this is set to false then the motors will turn on!
//#define PRINT_LOW_VOLTAGE_WARNING

/* PINOUT ASSIGNMENTS */
const int PIN_LOW_VOLT_MON = A0;
const int LOW_VOLT_LED_ENABLE = B00100000;
const int LOW_VOLT_LED_DISABLE = B11011111;
const int ESC_4x_ENABLE = B11110000;

/* CONSTANTS */
// Array indices
static const int x = 0, y = 1, z = 2;
static const int roll = 0, pitch = 1, yaw = 2;
// Quadcopter settings
static const int MAX_ANGLE_DEG = 7;
static const int MAX_ANGLE_DEG_YAW = 25;
// RX receiver
static const int MIN_RX_THROTTLE = 1008;
static const int MAX_RX_THROTTLE = 1800;
static const int MIN_RX_ANGLE_PULSE = 1050; // actual: 1004
static const int MAX_RX_ANGLE_PULSE = 1950; // actual: 1964
static const int RX_ACTIVATION_THROTTLE = 1950; // throttle needed to be reached to arm the motors
// ESC input range
static const int MIN_ESC_PULSE = 1000;
static const int MAX_ALLOWABLE_PULSE = 1950;  // maximum pulse allowed to be outputted to any ESC
static const int MIN_ALLOWABLE_PULSE = 1030;  // minimum pulse allowed to be outputted to any ESC when throttle is on
// Timekeeping
static const int REFRESH_RATE_MICROS = 6000;  // time interval (in micros) to maintain 166 Hz loop
// Low voltage battery monitor
static const float VOLTAGE_MIN = 1.9; // corresponds to 11.1 V static battery voltage
// IMU sensitivities
static const float GYRO_SENSITIVITY = 65.5;     // +- 500 deg/s gyroscope sensitivity
static const float ACC_SENSITIVITY = 4096;    // +- 8g accelerometer sensitivity
// IMU register addresses 
static const int MPU6050_addr = 0x68;   // I2C slave address of MPU-6050
static const int PWR_MGMT_1   = 0x6B;   // power register
static const int ACCEL_XOUT_H = 0x3B;   // first accelerometer data register
static const int GYRO_XOUT_H  = 0x43;   // first gyroscope data register
static const int GYRO_CONFIG  = 0x1B;   // gyroscope configuration register
// filter coefficients
static const float COMP_FILTER_ALPHA = 0.98;
static const float ACC_LP_FILTER_ALPHA = 0.08;
static const float RX_LP_FILTER_ALPHA = 0.15;
static const float ERROR_LP_FILTER_ALPHA = 0.18;
static const float LP_MED_COMBO_FILTER_ALPHA = 0.9;
static const LowPassFilter acc_roll_lp_filter(ACC_LP_FILTER_ALPHA);
static const LowPassFilter acc_pitch_lp_filter(ACC_LP_FILTER_ALPHA);
static const LowPassFilter rx_throttle_lp_filter(RX_LP_FILTER_ALPHA);
static const LowPassFilter rx_roll_lp_filter(RX_LP_FILTER_ALPHA);
static const LowPassFilter rx_pitch_lp_filter(RX_LP_FILTER_ALPHA);
static const LowPassFilter rx_yaw_lp_filter(RX_LP_FILTER_ALPHA);
static const LowPassFilter roll_error_filter(ERROR_LP_FILTER_ALPHA);
static const LowPassFilter pitch_error_filter(ERROR_LP_FILTER_ALPHA);

// PID controller variables
static const int PID_ACTIVATION_THROTTLE = 1300;  // 1200
static const int I_TERM_ACTIVATION_THROTTLE = 1400;
//static const float P_GAIN_ROLL_PITCH = 2.5;  // 2
//static const float D_GAIN_ROLL_PITCH = 120; // 40
//static const float I_GAIN_ROLL_PITCH = 0;  // 0.01
//static const float P_GAIN_YAW = 0;  // 2
//static const float I_GAIN_YAW = 0;
//static PIDController pid_controller_roll(P_GAIN_ROLL_PITCH, I_GAIN_ROLL_PITCH, D_GAIN_ROLL_PITCH);
//static PIDController pid_controller_pitch(P_GAIN_ROLL_PITCH, I_GAIN_ROLL_PITCH, D_GAIN_ROLL_PITCH);
//static PIDController pid_controller_yaw(P_GAIN_YAW, I_GAIN_YAW, 0);

static float i_angle_output[3], i_rate_output[3];
static float error_prior_angle[3], error_prior_rate[3];
static const float i_thresh = 100;
static float max_output = 300;
static float pid_output[3];

/* GLOBALS */
static long loop_iter = 0;
// RX
byte lastChannel1, lastChannel2, lastChannel3, lastChannel4, lastChannel8; // determines which channel was last enabled
unsigned long timer1, timer2, timer3, timer4, timer8, currentTime;   // tracks time of rising RX pulse
unsigned int rx_roll_pulse, rx_pitch_pulse, rx_throttle_pulse, rx_yaw_pulse, rx_aux_2;   // RX input pulse
unsigned int bounded_roll_pulse, bounded_pitch_pulse, bounded_yaw_pulse, bounded_throttle_pulse;
static float rx_deg[3];
// ESC
unsigned int esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse;  // time duration of ESC pulses (in microsec)
unsigned long loop_timer;  // loop timer to maintain 250Hz refresh rate
// IMU raw data
static boolean IMU_found = true;
static int16_t gyro_bias[3];   // average gyro offset in each DOF
static int16_t gyro_raw[3];      // raw gyro sensor values
static int16_t acc_bias[3];      // average acc offset in each DOF
static int16_t acc_raw[3];         // converted values
// IMU calculated data
static float gyro[3];     // angles derived from gyro readings
static float gyro_dot[3]; // angular velocities derived from gyro readings
static float acc_in_g[3]; // acc values converted to units of G's
static float acc_radians[2];  // acc vals in radians
static float acc[2];      // angles derived from acc readings
static float imu_deg[3];  // angles derived from gyro/acc fusion
static boolean startingGyroAnglesSet = false;
// filter input/output
static const int ACC_BUFF_SIZE = 3;
static float acc_roll_buffer[ACC_BUFF_SIZE], acc_pitch_buffer[ACC_BUFF_SIZE];   // queue that keeps track of latest 3 readings from accelerometer
static float low_pass[2];
static float low_pass_prior[2];
static float median_pass[2];

/* TIMEKEEPING */
static float time_elapsed = 0.0;      // flight time elapsed
unsigned long previous_time = 0;      // last time of loop start
unsigned long current_time;           // current time of loop start
static float delta_t = 0.0;           // time between loops in sec

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
  PCMSK0 |= (1 << PCINT4);  // enable interrupt on arduino pin 12

  Serial.begin(115200);
  Wire.begin();     // initiate Wire library for IMU
  scanI2CDevices(); // identify all connected devices that use I2C  
  while(!IMU_found) {  // halt execution and blink LED if IMU is not located
    blink_led(1, 500);
  }
  initializeIMU();  // power up, set IMU sensitivities, and calculate gyro offset
  
  // pre-loop inhibit:
  // wait until receiver inputs are valid and in lowest position before starting
  // send pulse to ESC to prevent from beeping
  wait_for_motor_arming();

  while(rx_throttle_pulse > MIN_RX_THROTTLE+4 || rx_throttle_pulse < MIN_ESC_PULSE) {
    if(rx_yaw_pulse > MAX_RX_ANGLE_PULSE || rx_yaw_pulse < MIN_RX_ANGLE_PULSE) {  // disarm motors and wait to be rearmed
      PORTB &= LOW_VOLT_LED_DISABLE;  // turn off led
      wait_for_motor_arming();        // wait for max throttle input
    }
    send_minimum_esc_pulse();
    delay(25);
  }

  blink_led(3, 125);
}


void loop() {
  // HALT
//  while(loop_iter > 2500);  // 15s
  
  /* MAINTAIN 166 Hz LOOP FREQUENCY*/
  while(micros() <= loop_timer + REFRESH_RATE_MICROS);
  loop_timer = micros(); 

  /* CHECK FOR MOTOR DISARM */
  if(rx_throttle_pulse < MIN_RX_THROTTLE+12) {
    if(rx_yaw_pulse > MAX_RX_ANGLE_PULSE || rx_yaw_pulse < MIN_RX_ANGLE_PULSE) {  // disarm motors and wait to be rearmed
      wait_for_motor_arming();        // wait for max throttle input
    }
  }

  /* LOW VOLTAGE BATTERY MONITOR */
  const int sensorValue = analogRead(PIN_LOW_VOLT_MON); // read value of pin A0 (value in range 0-1023)
  const float voltage = sensorValue * (5.0 / 1023.0);   // convert sensorValue to voltage between 0V-5V
  if((voltage < VOLTAGE_MIN) && (voltage > VOLTAGE_MIN-0.5)) {
    #ifdef PRINT_LOW_VOLTAGE_WARNING
      Serial.print("WARNING: Low voltage - ");  Serial.println(voltage);
    #endif
    PORTB |= LOW_VOLT_LED_ENABLE;   // set digital output 13 HIGH
  } else {
    PORTB &= LOW_VOLT_LED_DISABLE; // set digital output 13 LOW
  }

  /* TIMEKEEPING */
  current_time = micros();
  if(loop_iter > 0) {  // if not first loop iteration
    delta_t = ((float) (current_time - previous_time)) * 1E-6;   // calculate time passed in seconds
  } 
  time_elapsed += delta_t;
  previous_time = current_time;

  /* READ IMU REGISTERS */
  getIMURegisterData();
  
  /* GYRO CALCULATIONS */
  // convert from 16-bit integer to angular velocity in deg/s
  gyro_dot[roll] = ((float) gyro_raw[y]) / GYRO_SENSITIVITY;
  gyro_dot[pitch] = ((float) gyro_raw[x]) / GYRO_SENSITIVITY;
  gyro_dot[yaw] = ((float) gyro_raw[z]) / GYRO_SENSITIVITY;

  // flip roll and yaw axis
  gyro_dot[roll] *= -1; // flip gyro roll axis
  gyro_dot[yaw] *= -1;  // flip gyro yaw axis

  gyro[roll] += gyro_dot[roll] * delta_t;
  gyro[pitch] += gyro_dot[pitch] * delta_t;
  gyro[yaw] += gyro_dot[yaw] * delta_t;

  // integrate to obtain roll, pitch, and yaw values
  imu_deg[roll] += gyro_dot[roll] * delta_t;
  imu_deg[pitch] += gyro_dot[pitch] * delta_t;
  imu_deg[yaw] += gyro_dot[yaw] * delta_t;

  // accomodate for yaw movement
  const double sine_of_yaw = sin(gyro_dot[yaw] * delta_t * 0.01744);
  gyro[roll] -= gyro[pitch] * sine_of_yaw;
  gyro[pitch] += gyro[roll] * sine_of_yaw;
  imu_deg[roll] -= imu_deg[pitch] * sine_of_yaw;
  imu_deg[pitch] += imu_deg[roll] * sine_of_yaw;

  /* ACCELEROMETER CALCULATIONS */
  acc_in_g[x] = ((float) acc_raw[x])/ACC_SENSITIVITY; // convert acc x from int to g
  acc_in_g[y] = ((float) acc_raw[y])/ACC_SENSITIVITY; // convert acc y from int to g
  acc_in_g[z] = ((float) acc_raw[z])/ACC_SENSITIVITY; // convert acc z from int to g
  acc_radians[roll] = atan2(-acc_in_g[x], sqrt((acc_in_g[y]*acc_in_g[y]) + (acc_in_g[z]*acc_in_g[z])));  // calculate roll in radians
  acc_radians[pitch]  = atan2(acc_in_g[y], acc_in_g[z]);  // calculate pitch in radians 
  acc[roll] = acc_radians[roll] * RAD_TO_DEG;   // convert acc roll from radians to degrees
  acc[pitch] = acc_radians[pitch] * RAD_TO_DEG; // conver acc pitch from radians to degrees
  acc[roll] *= -1;  // flip accelerometer roll axis
  
  // compensate for offset IMU placement
  acc[roll]  += 2.2; 
  acc[pitch] -= 1.4;  // previously 3 at steady state

  // add current accelerometer readings to buffer
  enqueue(acc_roll_buffer, ACC_BUFF_SIZE, acc[roll]);  
  enqueue(acc_pitch_buffer, ACC_BUFF_SIZE, acc[pitch]);

  // apply low-pass & median filter to accelerometer data
  if(loop_iter > 1){
    median(acc_roll_buffer[0], acc_roll_buffer[1], acc_roll_buffer[2], median_pass[roll]);
    median(acc_pitch_buffer[0], acc_pitch_buffer[1], acc_pitch_buffer[2], median_pass[pitch]);
    acc[roll] = (LP_MED_COMBO_FILTER_ALPHA * acc_roll_lp_filter.output(acc[roll])) + ((1 - LP_MED_COMBO_FILTER_ALPHA) * median_pass[roll]);
    acc[pitch] = (LP_MED_COMBO_FILTER_ALPHA * acc_pitch_lp_filter.output(acc[pitch])) + ((1 - LP_MED_COMBO_FILTER_ALPHA) * median_pass[pitch]);
  } else {
    median_pass[roll] = acc[roll];
    median_pass[pitch] = acc[pitch];
  }
 
  
  /* GYRO/ACC FUSION CALCULATION */
  if(startingGyroAnglesSet) {
    // use complementary filter to fuse sensor readings
    imu_deg[roll] = (COMP_FILTER_ALPHA*imu_deg[roll]) + (1-COMP_FILTER_ALPHA)*acc[roll]; 
    imu_deg[pitch] = (COMP_FILTER_ALPHA*imu_deg[pitch]) + (1-COMP_FILTER_ALPHA)*acc[pitch];   
  } else {
    // accomodate for any starting angular offset due to unlevel surface
    gyro[roll] = acc[roll];
    gyro[pitch] = acc[pitch];
    imu_deg[roll] = gyro[roll];
    imu_deg[pitch] = gyro[pitch];
    startingGyroAnglesSet = true;
  }

  // transfer roll/pitch during yaw movement
//  float temp_roll = imu_deg[roll];
//  imu_deg[roll] -= imu_deg[pitch] * sin(imu_deg[yaw] * DEG_TO_RAD);
//  imu_deg[pitch] += temp_roll * sin(imu_deg[yaw] * DEG_TO_RAD);

  #ifdef DEBUG_ACC_ANGLES
    if (loop_iter % PRINT_RATE == 0)  log_string(acc[roll], acc[pitch], acc[yaw]);
  #endif

  #ifdef DEBUG_GYRO_ANGLES
    if (loop_iter % PRINT_RATE == 0)  log_string(gyro[roll], gyro[pitch], gyro[yaw]);
  #endif

  #ifdef DEBUG_ANGLES
    if (loop_iter % PRINT_RATE == 0)  log_string(imu_deg[roll], imu_deg[pitch], imu_deg[yaw]);
  #endif

  // cutoff RX inputs at upper and lower bound
  bounded_throttle_pulse = restrict_pulse_range(rx_throttle_pulse, MIN_RX_THROTTLE, MAX_RX_THROTTLE);
  bounded_roll_pulse = restrict_pulse_range(rx_roll_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE);
  bounded_pitch_pulse = restrict_pulse_range(rx_pitch_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE);
  bounded_yaw_pulse = restrict_pulse_range(rx_yaw_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE);

  // filter RX inputs
  bounded_throttle_pulse = rx_throttle_lp_filter.output(bounded_throttle_pulse);
  bounded_roll_pulse = rx_roll_lp_filter.output(bounded_roll_pulse);
  bounded_pitch_pulse = rx_pitch_lp_filter.output(bounded_pitch_pulse);
  bounded_yaw_pulse = rx_yaw_lp_filter.output(bounded_yaw_pulse);

  #ifdef DEBUG_RAW_RX_PULSE
    if (loop_iter % PRINT_RATE == 0)  log_string(rx_throttle_pulse, rx_roll_pulse, rx_pitch_pulse, rx_yaw_pulse);
  #endif

  #ifdef DEBUG_FILTERED_RX_PULSE
    if (loop_iter % PRINT_RATE == 0)  log_string(bounded_throttle_pulse, bounded_roll_pulse, bounded_pitch_pulse, bounded_yaw_pulse);
  #endif

  // convert rx angle inputs from microseconds to degrees
  rx_deg[roll] = theta_pulse_to_deg(bounded_roll_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  rx_deg[pitch] = theta_pulse_to_deg(bounded_pitch_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
  rx_deg[yaw] = theta_pulse_to_deg(bounded_yaw_pulse, MIN_RX_ANGLE_PULSE, MAX_RX_ANGLE_PULSE, -MAX_ANGLE_DEG_YAW, MAX_ANGLE_DEG_YAW);

  #ifdef DEBUG_RX_DEG
    if (loop_iter % PRINT_RATE == 0)  log_string(rx_deg[roll], rx_deg[pitch], rx_deg[yaw]);
  #endif
  
//  float roll_error = imu_deg[roll] - rx_deg[roll];
//  float pitch_error = imu_deg[pitch] - rx_deg[pitch];
//  float yaw_error = imu_deg[yaw] - rx_deg[yaw];
//
//  roll_error = roll_error_filter.output(roll_error);
//  pitch_error = pitch_error_filter.output(pitch_error);

  /* PID CALCULATIONS */  
//  bool disable_i = bounded_throttle_pulse < I_TERM_ACTIVATION_THROTTLE;
//  const float roll_pid = pid_controller(imu_deg[roll], rx_deg[roll], P_GAIN_ROLL_PITCH, I_GAIN_ROLL_PITCH, D_GAIN_ROLL_PITCH, roll);
//  const float pitch_pid = pid_controller(imu_deg[pitch], rx_deg[pitch], P_GAIN_ROLL_PITCH, I_GAIN_ROLL_PITCH, D_GAIN_ROLL_PITCH, pitch);
//  const float yaw_pid = pid_controller(imu_deg[yaw], rx_deg[yaw], P_GAIN_YAW, I_GAIN_YAW, 0, yaw);

  calculate_pid(rx_deg);
  
  #ifdef DEBUG_PID
    if (loop_iter % PRINT_RATE == 0)  log_string(pid_output[roll], pid_output[pitch], pid_output[yaw]);
  #endif

  /* SEND ESC SIGNALS */
  // set esc input pulse to the throttle pulse from RX
  // activate PID when motors are spinning above a certain threshold
  if(rx_throttle_pulse > PID_ACTIVATION_THROTTLE) {
    esc_1_pulse = bounded_throttle_pulse - pid_output[roll]; // - pid_output[pitch] - pid_output[yaw]; // - roll, - pitch, - yaw
    esc_2_pulse = bounded_throttle_pulse - pid_output[roll]; // + pid_output[pitch] + pid_output[yaw]; // - roll, + pitch, + yaw
    esc_3_pulse = bounded_throttle_pulse + pid_output[roll]; // + pid_output[pitch] - pid_output[yaw]; // + roll, + pitch, - yaw
    esc_4_pulse = bounded_throttle_pulse + pid_output[roll]; // - pid_output[pitch] + pid_output[yaw]; // + roll, - pitch, + yaw
  } else {
    esc_1_pulse = bounded_throttle_pulse;
    esc_2_pulse = bounded_throttle_pulse;
    esc_3_pulse = bounded_throttle_pulse;
    esc_4_pulse = bounded_throttle_pulse;
    reset_pid_controllers();
  }
  
  /* ESC CORRECTIONS */
  // determine the maximum ESC pulse
  int16_t maximum_esc_pulse = esc_1_pulse;
  if(esc_2_pulse > maximum_esc_pulse) { maximum_esc_pulse = esc_2_pulse;  }
  if(esc_3_pulse > maximum_esc_pulse) { maximum_esc_pulse = esc_3_pulse;  }
  if(esc_4_pulse > maximum_esc_pulse) { maximum_esc_pulse = esc_4_pulse;  }

  // keep ESC pulse under MAX_ALLOWABLE_PULSE
  if(maximum_esc_pulse > MAX_ALLOWABLE_PULSE){
    const float difference = maximum_esc_pulse - MAX_ALLOWABLE_PULSE;
    esc_1_pulse -= difference;
    esc_2_pulse -= difference;
    esc_3_pulse -= difference;
    esc_4_pulse -= difference;
  }

  // determine the minimum ESC pulse
  int16_t minimum_esc_pulse = esc_1_pulse;
  if(esc_2_pulse < minimum_esc_pulse) { minimum_esc_pulse = esc_2_pulse;  }
  if(esc_3_pulse < minimum_esc_pulse) { minimum_esc_pulse = esc_3_pulse;  }
  if(esc_4_pulse < minimum_esc_pulse) { minimum_esc_pulse = esc_4_pulse;  }

  // keep ESC pulse over MIN_ALLOWABLE_PULSE if user is throttling
  if(minimum_esc_pulse < MIN_ALLOWABLE_PULSE){
    const float difference = MIN_ALLOWABLE_PULSE - minimum_esc_pulse;
    esc_1_pulse += difference;
    esc_2_pulse += difference;
    esc_3_pulse += difference;
    esc_4_pulse += difference;
  }

  #ifdef DEBUG_ESC
    if (loop_iter % PRINT_RATE == 0)  log_string(esc_1_pulse, esc_2_pulse, esc_3_pulse, esc_4_pulse);
  #endif
  
  // TODO: compensate ESC pulse for battery voltage drop & PID corrections
  
  // if user is not throttling, then cut off motor power
  if(rx_throttle_pulse < MIN_ALLOWABLE_PULSE || DISABLE_MOTORS) {
    send_minimum_esc_pulse();
  } else {
    send_esc_pulse();
  }
  loop_iter++;

  log_string(imu_deg[roll], rx_deg[roll]);
}

/*  END LOOP  */
float angle_controller(const float input, const float setpoint, const float p_gain, const float i_gain, const float d_gain, const int axis) {
  float error = input - setpoint;
  return pid(p_gain, i_gain, d_gain, error, &error_prior_angle[axis], &i_angle_output[axis]);
}

float rate_controller(const float input, const float setpoint, const float p_gain, const float i_gain, const float d_gain, const int axis) {
  float error = input - setpoint;
  return pid(p_gain, i_gain, d_gain, error, &error_prior_rate[axis], &i_rate_output[axis]);
}

float pid(const float p_gain, const float i_gain, const float d_gain, const float error, float* error_prior, float* i_output) {
  if(bounded_throttle_pulse > I_TERM_ACTIVATION_THROTTLE) {
    *i_output += i_gain * error;
  } else {
    *i_output = 0;
  }
    
  // anti-windup; prevent I term from getting too big
  if(*i_output > i_thresh) *i_output = i_thresh;
  else if(*i_output < -i_thresh) *i_output = -i_thresh;
  
  *error_prior = error;
  
  // combine output
  float output = (p_gain * error) + (*i_output) + (d_gain * (error - *error_prior));
  
  if(output > max_output) output = max_output;
  else if(output < -max_output) output = -max_output;
  
  return output;
}

// angle controller gains
const float P_GAIN_ROLL_PITCH_ANGLE = 0;
const float I_GAIN_ROLL_PITCH_ANGLE = 0;
const float D_GAIN_ROLL_PITCH_ANGLE = 0;
const float P_GAIN_YAW_ANGLE = 0;
const float I_GAIN_YAW_ANGLE = 0;
// rate controller gains
const float P_GAIN_ROLL_PITCH_RATE = 0;
const float I_GAIN_ROLL_PITCH_RATE = 0;
const float D_GAIN_ROLL_PITCH_RATE = 0;
const float P_GAIN_YAW_RATE = 0;
const float I_GAIN_YAW_RATE = 0;

float calculate_pid(float rx_deg[]) {
  // Angle Controller
  // ----------------
  // input: imu_deg (deg)
  // setpoint: rx_deg (deg)
  float roll_angle_pid = angle_controller(imu_deg[roll], rx_deg[roll], P_GAIN_ROLL_PITCH_ANGLE, I_GAIN_ROLL_PITCH_ANGLE, D_GAIN_ROLL_PITCH_ANGLE, roll);
  float pitch_angle_pid = angle_controller(imu_deg[pitch], rx_deg[pitch], P_GAIN_ROLL_PITCH_ANGLE, I_GAIN_ROLL_PITCH_ANGLE, D_GAIN_ROLL_PITCH_ANGLE, pitch);
  float yaw_angle_pid = angle_controller(imu_deg[yaw], rx_deg[yaw], P_GAIN_YAW_ANGLE, I_GAIN_YAW_ANGLE, 0, yaw);
  
  // Rate Controller
  // ----------------
  // input: gyro_dot (deg/s)
  // setpoint: output from angle controller (deg/s)
//  float roll_rate_pid = angle_controller(gyro_dot[roll], roll_angle_pid, P_GAIN_ROLL_PITCH, I_GAIN_ROLL_PITCH, D_GAIN_ROLL_PITCH, roll);
  float roll_rate_pid = rate_controller(gyro_dot[roll], 0, P_GAIN_ROLL_PITCH_RATE, I_GAIN_ROLL_PITCH_RATE, D_GAIN_ROLL_PITCH_RATE, roll);
  float pitch_rate_pid = rate_controller(gyro_dot[pitch], 0, P_GAIN_ROLL_PITCH_RATE, I_GAIN_ROLL_PITCH_RATE, D_GAIN_ROLL_PITCH_RATE, pitch);
  float yaw_rate_pid = rate_controller(gyro_dot[yaw], 0, P_GAIN_YAW_RATE, I_GAIN_YAW_RATE, 0, yaw);

  pid_output[roll] = roll_rate_pid;
  pid_output[pitch] = pitch_rate_pid;
  pid_output[yaw] = yaw_rate_pid;
}

  
void reset_pid_controllers() {
  pid_output[roll] = 0;  
  pid_output[pitch] = 0; 
  pid_output[yaw] = 0; 
  i_angle_output[roll] = 0;  
  i_angle_output[pitch] = 0; 
  i_angle_output[yaw] = 0;
  i_rate_output[roll] = 0;  
  i_rate_output[pitch] = 0; 
  i_rate_output[yaw] = 0;
  error_prior_angle[roll] = 0;
  error_prior_angle[pitch] = 0;
  error_prior_angle[yaw] = 0;
  error_prior_rate[roll] = 0;
  error_prior_rate[pitch] = 0;
  error_prior_rate[yaw] = 0;
}

// sends pulse to esc with the values of esc_1_pulse, esc_2_pulse, esc_3_pulse, and esc_4_pulse
void send_esc_pulse() {  
  unsigned long offset_timer = micros();  // remember the current time
  PORTD |= ESC_4x_ENABLE;   // set digital pins 4-7 to HIGH
  /* 
   * HERE THERE IS A 1000us-2000us gap of time
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
  PRINTLN_BOOTUP_SEQ("wait_for_motor_arming()");
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
  acc_raw[x] = Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3B (ACCEL_XOUT_L)
  acc_raw[y] = Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc_raw[z] = Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  // skip the 2 temperature registers
  Wire.read()<<8|Wire.read();

  // read gyroscope registers
  gyro_raw[x] = Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyro_raw[y] = Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyro_raw[z] = Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // subtract the calibration offsets
  gyro_raw[x] -= gyro_bias[x];
  gyro_raw[y] -= gyro_bias[y];     
  gyro_raw[z] -= gyro_bias[z];
}

// take the average of the first 2000 gyroscope readings
void calculateSensorBias() {
  PRINTLN_BOOTUP_SEQ("Initializing gyroscope calibration...");
  long gyro_raw[3], acc_raw[3];
  gyro_raw[x] = gyro_raw[y] = gyro_raw[z] = 0;
  acc_raw[x] = acc_raw[y] = acc_raw[z] = 0;
  const int calibrationIterations = 2000;
  int percent = 0;

  bool led_on = true;

  // calculate offsets
  for(int i = 0; i < calibrationIterations; i++) {
    if(i % (calibrationIterations/10) == 0) {
      percent += 10;
      PRINT_BOOTUP_SEQ(percent); PRINTLN_BOOTUP_SEQ("%");
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
    acc_raw[x] += Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3B (ACCEL_XOUT_L)
    acc_raw[y] += Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    acc_raw[z] += Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    
    // skip the 2 temperature registers
    Wire.read()<<8|Wire.read();
    
    // read gyroscope registers
    gyro_raw[x] += Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
    gyro_raw[y] += Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
    gyro_raw[z] += Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
    delay(3);
  }
  
  acc_bias[x] = (float)(acc_raw[x]) / calibrationIterations;
  acc_bias[y] = (float)(acc_raw[y]) / calibrationIterations;
  acc_bias[z] = (float)(acc_raw[z]) / calibrationIterations;
  gyro_bias[x] = (float)(gyro_raw[x]) / calibrationIterations;
  gyro_bias[y] = (float)(gyro_raw[y]) / calibrationIterations;
  gyro_bias[z] = (float)(gyro_raw[z]) / calibrationIterations;
}

void scanI2CDevices() {
  PRINTLN_BOOTUP_SEQ("Scanning for I2C devices...");
  byte error, address;
  int numDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0) {    // success
      PRINTLN_BOOTUP_SEQ("I2C device found at address 0x");
      if(address < 16)
        PRINTLN_BOOTUP_SEQ("0");
      PRINTLN_BOOTUP_SEQ_HEX(address);
      numDevices++;
    } else if(error == 4) {   // error
      PRINTLN_BOOTUP_SEQ("Unknown error at address 0x");  
      if(address < 16)
        PRINTLN_BOOTUP_SEQ("0");
      PRINTLN_BOOTUP_SEQ_HEX(address);
    }
  }

  if(numDevices == 0) {
    PRINTLN_BOOTUP_SEQ("No I2C devices found.");  
    IMU_found = false;
  } else {
    PRINTLN_BOOTUP_SEQ("Finished scanning I2C devices.");
  }
}

void enqueue(float buffer[], int length, float new_val) {
  for(int i = 0; i < length-1; i++) {
    buffer[i] = buffer[i+1];
  }
  buffer[length-1] = new_val;
}

// assign to output the median of the 3 inputs
void median(const float in1, const float in2, const float in3, float& output) {
  float lo = in1;
  float med = in2;
  float hi = in3;
  if(lo > med) swap(&lo, &med);
  if(med > hi) swap(&med, &hi);
  if(lo > med) swap(&lo, &med);
  output = med;
}

void swap(float* a, float* b) {
  float temp = *b;
  *b = *a;
  *a = temp;
}

void blink_led(int n, int t) {
  for(int i = 0; i < n; i++) {
    PORTB |= LOW_VOLT_LED_ENABLE;
    delay(t);
    PORTB &= LOW_VOLT_LED_DISABLE;
    delay(t);
  }
}

float theta_pulse_to_deg(float rx_input_pulse, float from_low, float from_hi, float to_low, float to_hi) {
  //if(rx_input_pulse >= 1492 && rx_input_pulse <= 1508) { rx_input_pulse = 1500; }   // set degree to 0.0 if pulse is around 1500
  return float((rx_input_pulse - from_low) * (to_hi - to_low)) / (from_hi - from_low) + to_low;
}

int restrict_pulse_range(int pulse, int low, int high) {
  if(pulse < low) { 
    pulse = low;
  } else if(pulse > high) { 
    pulse = high; 
  }
  return pulse;
}

/* PRINT FUNCTIONS */

void readAndPrintMPURegister(const int startingRegisterNumber, const int numRegisters) {
  PRINT_BOOTUP_SEQ("<Register dump>: "); PRINTLN_BOOTUP_SEQ(numRegisters);
  PRINT_BOOTUP_SEQ(" register(s), starting at 0x"); PRINTLN_BOOTUP_SEQ_HEX(startingRegisterNumber);
  
  // queue the registers for reading
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(startingRegisterNumber);    // select register
  Wire.endTransmission();
  
  // read requested registers
  Wire.requestFrom(MPU6050_addr, numRegisters, true); // request read from 2 registers
  for(int i = 0; i < numRegisters; i++) {
    byte REGISTER_VALUE = Wire.read();
    PRINT_BOOTUP_SEQ("Register 0x"); PRINT_BOOTUP_SEQ_HEX(startingRegisterNumber+i);
    PRINT_BOOTUP_SEQ(":\t");  PRINTLN_BOOTUP_SEQ(REGISTER_VALUE);
  }
  PRINTLN_BOOTUP_SEQ();
}

void log_string(float f1, float f2) {
  String str = String(f1);
  str += "\t";
  str += String(f2);
  Serial.println(str);  
}

void log_string(float f1, float f2, float f3) {
  String str = String(f1);
  str += "\t";
  Serial.print(str);
  log_string(f2, f3);  
}

void log_string(float f1, float f2, float f3, float f4) {
  String str = String(f1);
  str += "\t";
  Serial.print(str);
  log_string(f2, f3, f4);
}
