#include "MPU6050.h"
#include <MadgwickAHRS.h>
#include "drone_motor.h"

int16_t ax, ay, az;//加速度 int16_tは2バイトの符号付き整数
int16_t gx, gy, gz;//角速度 同上
float ROLL, PITCH, YAW;

//select pwm_pin
const int FR_motor_pin = 2;
const int FL_motor_pin = 17;
const int BR_motor_pin = 26;
const int BL_motor_pin = 32;

MPU6050 accelgyro;

Madgwick MadgwickFilter;

drone_motor FR_motor(FR_motor_pin);
drone_motor FL_motor(FL_motor_pin);
drone_motor BR_motor(BR_motor_pin);
drone_motor BL_motor(BL_motor_pin);

void mpu6050_calibration();
void get_mpu6050data();
void set_motor_speed();

void setup(){
    Wire.begin();
    Serial.begin(115200);
    while (!Serial) delay(10);
    accelgyro.initialize();//I2Cデバイスの初期化
    delay(300);
    MadgwickFilter.begin(100);//フィルタのサンプリングを100Hzで

    // =====bluetooth setup=========
    PS4.begin("BC:FE:D9:CF:B6:5D");
    Serial.println("Ready.");
    delay(500);
    while(!PS4.isConnected()){
        Serial.println("wait connecting ds4...");
        delay(1000);
    }
    Serial.println("ds4 connected!");
}

void loop(){
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  MadgwickFilter.updateIMU(gx / 131.0, gy / 131.0, gz / 131.0, ax / 16384.0, ay / 16384.0, az / 16384.0);
  ROLL = MadgwickFilter.getRoll();
  PITCH = MadgwickFilter.getPitch();
  YAW  = MadgwickFilter.getYaw();
  Serial.print(ROLL); Serial.print(",");
  Serial.print(PITCH); Serial.print(",");
  Serial.print(YAW);
  Serial.print("\n");
  delay(10);
}


