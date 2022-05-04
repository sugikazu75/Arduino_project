#include <Wire.h>
#include "mpu6050_init.h"
#include <MadgwickAHRS.h>
Madgwick MadgwickFilter;

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

double ROLL, PITCH, YAW;
double offset_ROLL=0.0, offset_PITCH=0.0, offset_YAW=0.0;   //取り付け誤差を打ち消す

void culcRotation();
void mpu6050_calib();

void setup() {
  Wire.begin(21, 22);   //sda, scl
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(115200);
  delay(100);  
  //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

  mpu6050_calib();
}

void loop() {
  calcRotation();
}

//加速度、ジャイロから角度を計算
void calcRotation(){
  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z;  
  //レジスタアドレス0x3Bから、計14バイト分のデータを出力するようMPU6050へ指示
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  //出力されたデータを読み込み、ビットシフト演算
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();

  MadgwickFilter.updateIMU(raw_gyro_x / 131.0, raw_gyro_y / 131.0, raw_gyro_z / 131.0, raw_acc_x / 16384.0, raw_acc_y / 16384.0, raw_acc_z / 16384.0);
  ROLL = MadgwickFilter.getRoll() - offset_ROLL;
  PITCH = MadgwickFilter.getPitch() - offset_PITCH;
  YAW  = MadgwickFilter.getYaw() - offset_YAW;
//  Serial.print(ROLL); Serial.print(",");
//  Serial.print(PITCH); Serial.print(",");
//  Serial.print(YAW);
//  Serial.print("\n");
}

void mpu6050_calib(){
  double sum_ROLL=0.0, sum_PITCH=0.0, sum_YAW=0.0;
  Serial.println("mpu6050 calibration");
  for(int i=0; i<3000; i++){
    calcRotation();
    sum_ROLL += ROLL;
    sum_PITCH += PITCH;
    sum_YAW += YAW;
  }
  offset_ROLL = sum_ROLL / 3000.0;
  offset_PITCH = sum_PITCH / 3000.0;
  offset_YAW = sum_YAW / 3000.0;  
//  Serial.print(offset_ROLL); Serial.print(",");
//  Serial.print(offset_PITCH); Serial.print(",");
//  Serial.print(offset_YAW);
  Serial.print("\ndone calibration\n");
  delay(1000);
}
