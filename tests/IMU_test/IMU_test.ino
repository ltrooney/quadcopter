// MPU6050 GY-521 Test

#include <Wire.h>
#include <Math.h>

// sensor settings
static const float GYRO_SENSITIVITY = 65.5;     // +- 500 deg/s sensitivity
static const int16_t ACC_SENSITIVITY = 4096;    // +- 8g sensitivity

// sensor data
static int16_t gyroOffs[3];   // average gyro offset in each DOF
static int16_t gyro[3];       // raw gyro sensor values
static int16_t acc[3];        // converted values

// calculated data
static float angularVelocities[3];    // angular velocities derived from gyro readings
static float gyroAngles[3];           // angles derived from gyro readings
static float accAngles[3];            // angles derived from acc readings
static float angles[3];               // angles derived from gyro/acc fusion

// timekeeping
static float timeElapsed = 0.0;       // flight time elapsed
unsigned long previous_time = 0.0;    // last time of loop start
unsigned long current_time;           // current time of loop start
static float delta_t;                 // time between loops in sec

// indices
static const int X = 0, Y = 1, Z = 2;
static const int ROLL = 0, PITCH = 1, YAW = 2;
  
// register addresses
static const int MPU6050_addr = 0x68;   // I2C slave address of MPU-6050
static const int PWR_MGMT_1   = 0x6B;   // power register
static const int ACCEL_XOUT_H = 0x3B;   // first accelerometer data register
static const int GYRO_XOUT_H  = 0x43;   // first gyroscope data register
static const int GYRO_CONFIG  = 0x1B;   // gyroscope configuration register

// conversion
const float millis_to_seconds = 1E-3;   // convert milliseconds to seconds

static boolean startingGyroAnglesSet = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();             // initiate Wire library
  scanI2CDevices();         // identify all connected devices that use I2C  
  initializeIMU();          // power up, set sensitivities, and calculate gyro offset
}


void loop() {
  // TODO: achieve more accuracy with gyro angle measurement
 
  // timekeeping
  current_time = millis();
  if(previous_time == 0.0)    // if first iteration of loop
    delta_t = 0.0;
  else
    delta_t = (current_time - previous_time) * millis_to_seconds;
  timeElapsed += delta_t;

  // get the raw gyro and acc values from the IMU
  getIMURegisterData();

  /*
  * PERFORM CALCULATIONS FROM GYRO 
  */
  // convert from integer to angular velocity in deg/s
  angularVelocities[ROLL] = ((float) gyro[Y]) / GYRO_SENSITIVITY;
  angularVelocities[PITCH] = ((float) gyro[X]) / GYRO_SENSITIVITY;
  angularVelocities[YAW] = ((float) gyro[Z]) / GYRO_SENSITIVITY;

//  if(angularVelocities[ROLL] < 0.5 && angularVelocities[ROLL] > -0.5) {
//    angularVelocities[ROLL] = 0.0;
//  }
//  if(angularVelocities[PITCH] < 0.5 && angularVelocities[PITCH] > -0.5) {
//    angularVelocities[PITCH] = 0.0;
//  }
//  if(angularVelocities[YAW] < 0.5 && angularVelocities[YAW] > -0.5) {
//    angularVelocities[YAW] = 0.0;
//  }

  // integrate to obtain roll, pitch, and yaw values
  angles[ROLL] += angularVelocities[ROLL] * delta_t;
  angles[PITCH] += angularVelocities[PITCH] * delta_t;
  angles[YAW] += angularVelocities[YAW] * delta_t;

  // accomodate for yaw
  const double tempR = angles[ROLL];
  angles[ROLL] -= angles[PITCH] * sin(angularVelocities[YAW] * delta_t * 0.01744);
  angles[PITCH] += angles[ROLL] * sin(angularVelocities[YAW] * delta_t * 0.01744);

  /*
  * PERFORM CALCULATIONS FROM ACCEL 
  */
  // convert from integer to g
  float accX_g = ((float) acc[X])/ACC_SENSITIVITY;
  float accY_g = ((float) acc[Y])/ACC_SENSITIVITY;
  float accZ_g = ((float) acc[Z])/ACC_SENSITIVITY;
  const float acc_PITCH_radians  = atan2(accY_g, accZ_g);
  const float acc_ROLL_radians = atan2(-accX_g, sqrt((accY_g*accY_g) + (accZ_g*accZ_g)));
  
  accAngles[ROLL] = acc_ROLL_radians * RAD_TO_DEG;
  accAngles[PITCH] = acc_PITCH_radians * RAD_TO_DEG; 

  accAngles[ROLL] -= 2.3;
  accAngles[PITCH] -= 2.0;
      
  /*
  * PERFORM FUSION CALCULATION 
  */
  if(startingGyroAnglesSet) {
    // use complementary filter to fuse readings
    angles[ROLL] = (0.999*angles[ROLL]) + (0.001*accAngles[ROLL]); 
    angles[PITCH] = (0.999*angles[PITCH]) + (0.001*accAngles[PITCH]); 
  } else {
    // accomodate for any starting angular offset due to unlevel surface
    angles[ROLL] = accAngles[ROLL];
    angles[PITCH] = accAngles[PITCH];
    startingGyroAnglesSet = true;
  }

  plot3Angles(angles[ROLL], angles[PITCH], angles[YAW]);


  /*
   * PRINT OUTPUT
   */
  //plotAngles();
  //Serial.print(timeElapsed);  Serial.print("\t");
//  plot3Angles(accAngles[ROLL], accAngles[PITCH], accAngles[YAW]);
//  plot3Angles(gyroAngles[ROLL], gyroAngles[PITCH], gyroAngles[YAW]);
//  plot3Angles(angularVelocities[ROLL], angularVelocities[PITCH], angularVelocities[YAW]);
//  plot3Angles(gyro[ROLL], gyro[PITCH], gyro[YAW]);
//  plot3Angles(angles[ROLL], angles[PITCH], angles[YAW]);
//  plot2Angles(angles[ROLL], angles[PITCH]);
  previous_time = current_time;
  delay(50); 
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
  calculateGyroOffset();  // calculate average error offset from gyro reading
}

void getIMURegisterData() {  
  // initialize data transmission
  Wire.beginTransmission(MPU6050_addr);
  
  Wire.write(ACCEL_XOUT_H);                 // begin reading at this register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr, 14, true); // request 14 registers for read access
 
  // read accelerometer registers
  acc[X] = Wire.read()<<8|Wire.read();      // 0x3B (ACCEL_XOUT_H) & 0x3B (ACCEL_XOUT_L)
  acc[Y] = Wire.read()<<8|Wire.read();      // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc[Z] = Wire.read()<<8|Wire.read();      // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  
  // skip the 2 temperature registers
  Wire.read()<<8|Wire.read();

  // read gyroscope registers
  gyro[X] = Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gyro[Y] = Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gyro[Z] = Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // subtract the calibration offsets
  gyro[X] -= gyroOffs[X];
  gyro[Y] -= gyroOffs[Y];     
  gyro[Z] -= gyroOffs[Z];
}

// take the average of the first 2000 gyroscope readings
void calculateGyroOffset() {
  Serial.println("Initializing gyroscope calibration...");
  
  long rawGyroX = 0, rawGyroY = 0, rawGyroZ = 0;
  const int calibrationIterations = 2000;
  int percent = 0;

  // calculate offsets
  for(int i = 0; i < calibrationIterations; i++) {
    if(i % (calibrationIterations/10) == 0) {
      percent += 10;
      Serial.print(percent); Serial.println("%");
    }
    Wire.beginTransmission(MPU6050_addr);
    Wire.write(GYRO_XOUT_H);                    // begin reading at first gyro register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_addr, 6, true);    // request all 6 gyro registers
    
    // read gyroscope registers
    rawGyroX += Wire.read()<<8|Wire.read();     // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
    rawGyroY += Wire.read()<<8|Wire.read();     // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
    rawGyroZ += Wire.read()<<8|Wire.read();     // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)
    delay(3);
  }
  
  Serial.println("gyroscope calibration values collected!");

  gyroOffs[X] = (float)(rawGyroX) / calibrationIterations;
  gyroOffs[Y] = (float)(rawGyroY) / calibrationIterations;
  gyroOffs[Z] = (float)(rawGyroZ) / calibrationIterations;

  // print the averages
  Serial.print("rawGyroX: "); Serial.println((float)(gyroOffs[X]) / GYRO_SENSITIVITY);
  Serial.print("rawGyroY: "); Serial.println((float)(gyroOffs[Y])/ GYRO_SENSITIVITY);
  Serial.print("rawGyroZ: "); Serial.println((float)(gyroOffs[Z]) / GYRO_SENSITIVITY);  
}

/* HELPERS */
void scanI2CDevices() {
  Serial.println("Scanning for I2C devices...");
  byte error, address;
  int numDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if(error == 0) {
      Serial.print("I2C device found at address 0x");
      if(address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      numDevices++;
    } else if(error == 4) {
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



/* PRINT FUNCTIONS */ 
void plot2Angles(float a1, float a2) {
  Serial.print(a1);
  Serial.print("\t"); Serial.println(a2);
}

void plot3Angles(float a1, float a2, float a3) {
  Serial.print(a1);
  Serial.print("\t"); Serial.print(a2);
  Serial.print("\t"); Serial.println(a3);
}

void printElapsedFlightTime() {
  Serial.print(timeElapsed);  Serial.println("s");;
}












