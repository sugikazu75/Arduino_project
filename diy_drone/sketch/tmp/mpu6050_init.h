#define MPU6050_ADDR         0x68 // MPU-6050 device address
//I2c書き込み
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

//i2C読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}
