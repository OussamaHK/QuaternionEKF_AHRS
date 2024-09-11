#include "QuaternionEKF.h"
#include <BMI160Gen.h>
#include "bmm150.h"
#include "bmm150_defs.h"
float roll, pitch, yaw, heading;//attitude angles
const float G = 9.807f; //gravity
//unsigned long CurrentMillis, PreviousMillis = 0;
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;

QuaternionEKF EKF;

volatile bool bmi_drdy = false;
volatile bool bmm150_drdy = false;
const int bmi160_i2c_addr = 0x69;
const int bmi160_interrupt_pin = 17;
const int bmm150_interrupt_pin = 2;
int aRange, gRange;// Raw measurements range
int gxRaw, gyRaw, gzRaw, axRaw, ayRaw, azRaw;//, tRaw;// raw gyro and acc values
float ax, ay, az;// accelerometer data
float gx, gy, gz;// gyroscope data
float mx, my, mz;// magnetometer data

//Interrupts handle functions
void bmi160_intr()
{
  bmi_drdy = true;
}

void bmm_drdy() {
  bmm150_drdy = true;
}


void setup() {
  // initialize device
  Serial.begin(115200); // initialize Serial communication
  while (!Serial && millis() < 2000) {}

  // initialize device
  Serial.println("Initializing IMU device...");

  // verify connection
  BMI160.begin(BMI160GenClass::I2C_MODE, bmi160_i2c_addr);// , bmi160_interrupt_pin??
  uint8_t dev_id = BMI160.getDeviceID();
  Serial.print("DEVICE ID: ");
  Serial.println(dev_id, HEX);

  // accelerometer and gyroscope range and rate commands
  BMI160.setAccelerometerRange(2);// 2*g
  BMI160.setAccelRate(BMI160_ACCEL_RATE_100HZ);
  BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_NORM);//digital low-pass filter mode
  //BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_NORM);//digital low-pass filter mode
  BMI160.setGyroRange(250);// 250 rad/s
  BMI160.setGyroRate(BMI160_GYRO_RATE_100HZ);

  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(3000);
  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
  BMI160.autoCalibrateGyroOffset();
  Serial.println(" Done");

  Serial.print("Starting Acceleration calibration and enabling offset compensation...");
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(BMI160.getAccelerometerOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(BMI160.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t"); // -2359
  Serial.print(BMI160.getAccelerometerOffset(Z_AXIS));
  Serial.print("\t"); // 1688
  Serial.print(BMI160.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(BMI160.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(BMI160.getGyroOffset(Z_AXIS));

  // interrupts (data ready pins)
  BMI160.setIntDataReadyEnabled(true);
  pinMode(bmi160_interrupt_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(bmi160_interrupt_pin), bmi160_intr, RISING);

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("Chip ID can not read!");
    while (1);
  } else {
    Serial.println("magnetometer detected!");
  }

  bmm.set_mag_rate(BMM150_DATA_RATE_25HZ);//set BMM150 ODR
  // interrupts (data ready pins)
  bmm.set_DRDY_bit(1); //Enable BMM150 interrupt
  pinMode(bmm150_interrupt_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(bmm150_interrupt_pin), bmm_drdy, RISING);
  //calibration
  Serial.println("  Start magnetometer calibration after 3 seconds.");
  delay(3000);
  Serial.println("Rotate in all directions");
  calibrate_mag(10000);
  Serial.print("\n\rCalibration done..");

  gRange = BMI160.getGyroRange();
  aRange = BMI160.getAccelerometerRange();
  //change errors covariance matrices
  //EKF.setAccelCovariance(0.025f); // also try 0.025f
  //EKF.setGyroCovariance(0.0001f); // also try 0.025f (in radians)
  //EKF.setGyroBiasCovariance(0.0001f);
  //EKF.setHeadingCovariance(0.0125f); // also try 0.025f, 0.0125f
}

void loop() {
  //CurrentMillis = millis();
  if (bmi_drdy == true) {
    //bmi_drdy = false;
    // read raw gyro measurements from device
    BMI160.readMotionSensor(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);
    // convert the raw gyro data to rad/second
    gx = convertRaw(gxRaw, gRange);//*PI/180;
    gy = convertRaw(gyRaw, gRange);//*PI/180;
    gz = convertRaw(gzRaw, gRange);//*PI/180;
    // convert the raw accelerometer data to meter/second/second
    ax = convertRaw(axRaw, aRange) * G;
    ay = convertRaw(ayRaw, aRange) * G;
    az = convertRaw(azRaw, aRange) * G;
    //bmm150_mag_data value;
    if (bmm150_drdy == true) {
      bmm.read_mag_data();
      // correct data with offset value (calibration)
      mx = (bmm.raw_mag_data.raw_datax - value_offset.x);
      my = (bmm.raw_mag_data.raw_datay - value_offset.y);
      mz = (bmm.raw_mag_data.raw_dataz - value_offset.z);
      //bmm150_drdy == false;
    }
    else {
      Serial.println("BMM data not ready");
      while (1);
    }
    // remap axes orientation: (done below in the EKF.update function)
//    ay = -ay;
//    az = -az;
//    gy = -gy;
//    gz = -gz;
//    Serial.print("ax: ");Serial.print(ax);Serial.print("\t");Serial.print("ay: ");Serial.print(ay);Serial.print("\t");Serial.print("az: ");Serial.print(az);Serial.print("\t");
//    Serial.println(" ");
//    Serial.print("yx: ");Serial.print(gx);Serial.print("\t");Serial.print("gy: ");Serial.print(gy);Serial.print("\t");Serial.print("gz: ");Serial.print(gz);Serial.print("\t");
//    Serial.println(" ");
//    Serial.print("mx: ");Serial.print(mx);Serial.print("\t");Serial.print("my: ");Serial.print(my);Serial.print("\t");Serial.print("mz: ");Serial.print(mz);Serial.print("\t");
//    Serial.println(" ");
//    delay(500);
    EKF.update(gx, -gy, -gz, ax, -ay, -az, mx, my, mz);
    Serial.print("Pitch: ");
    Serial.print(EKF.getPitch_rad()*180.0f/PI);
    Serial.print("\t");
    Serial.print("Roll: ");
    Serial.print(EKF.getRoll_rad()*180.0f/PI);
    Serial.print("\t");
    Serial.print("Yaw: ");
    Serial.print(EKF.getYaw_rad()*180.0f/PI);
    Serial.println("\t");
//    Serial.print("Heading: ");
//    Serial.print(EKF.getHeading_rad()*180.0f/PI);
//    Serial.println("\t");
    //Serial.println(" ");   
  }
  else {
    Serial.println("bmi data not ready");
    while (1);
  }
}
// function to convert raw values to measurement units
float convertRaw(int16_t raw, float range_abs)
{
  float slope;
  float val;

  /* Input range will be -32768 to 32767
     Output range must be -range_abs to range_abs */
  val = (float)raw;
  slope = (range_abs * 2.0f) / BMI160_SENSOR_RANGE;
  return -(range_abs) + slope * (val + BMI160_SENSOR_LOW);
}

int getAccelerometerRange()
{
  int range;

  switch (BMI160.getFullScaleAccelRange()) {
    case BMI160_ACCEL_RANGE_2G:
      range = 2;
      break;

    case BMI160_ACCEL_RANGE_4G:
      range = 4;
      break;

    case BMI160_ACCEL_RANGE_8G:
      range = 8;
      break;

    case BMI160_ACCEL_RANGE_16G:
    default:
      range = 16;
      break;
  }

  return range;
}

int getGyroRange()
{
  int range;

  switch (BMI160.getFullScaleGyroRange()) {
    case BMI160_GYRO_RANGE_2000:
      range = 2000;
      break;

    case BMI160_GYRO_RANGE_1000:
      range = 1000;
      break;

    case BMI160_GYRO_RANGE_500:
      range = 500;
      break;

    case BMI160_GYRO_RANGE_250:
      range = 250;
      break;

    case BMI160_GYRO_RANGE_125:
    default:
      range = 125;
      break;
  }

  return range;
}
// function to calibrate magnetometer
void calibrate_mag(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    Serial.print(".");
    delay(100);

  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
}
