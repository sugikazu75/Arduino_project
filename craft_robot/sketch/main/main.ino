#include <PS4Controller.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int power_pin = 2;
const int ds4_connect_pin = 4;
const int IN_PLUS_left = 14;
const int IN_MINUS_left = 27;
const int IN_PLUS_right = 26;
const int IN_MINUS_right = 25;
const int EN_left = 12;
const int EN_right = 33;
// mpu6050 sda pin = 21
// mpu6050 scl pin = 22 

class l298n_pwd{
  public:
    int EN, IN_PLUS, IN_MINUS, SPEED;
    l298n_pwd(int EN, int IN_PLUS, int IN_MINUS){
      this -> EN = EN;
      this -> IN_PLUS = IN_PLUS;
      this -> IN_MINUS = IN_MINUS;
      this -> SPEED = 0;
      pinMode(this -> IN_PLUS, OUTPUT);
      pinMode(this -> IN_MINUS, OUTPUT);
    }

    void set_speed(int cmd){
      this -> SPEED = cmd;
      if(cmd >= 0){
        digitalWrite(this -> IN_PLUS, HIGH);
        digitalWrite(this -> IN_MINUS, LOW);
      }
      else{
        digitalWrite(this -> IN_PLUS, LOW);
        digitalWrite(this -> IN_MINUS, HIGH);
      }
      analogWrite(this -> EN, abs(cmd));
    }
};

void getPS4buttons(void);

void settirespeed(void);

void mpu6050setup(void);

void getmpu6050(void);

l298n_pwd left(EN_left, IN_PLUS_left, IN_MINUS_left);
l298n_pwd right(EN_right, IN_PLUS_right, IN_MINUS_right);
Adafruit_MPU6050 mpu;

int L2=0, R2=0, CrossButton=0, L3X=0, L3Y=0, left_cmd=0, right_cmd=0;

void setup(){
  // ========serial setup=================
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  // =======light setup===========
  pinMode(power_pin, OUTPUT);
  pinMode(ds4_connect_pin, OUTPUT);
  digitalWrite(power_pin, HIGH);

  // =====bluetooth setup=========
  PS4.begin("BC:FE:D9:CF:B6:5D");
  Serial.println("Ready.");
  delay(500);
  while(!PS4.isConnected()){
    Serial.println("wait connecting ds4...");
    delay(1000);
  }
  Serial.println("ds4 connected!");
  digitalWrite(4, HIGH);

  // ===========mpu6050 setup=======
  mpu6050setup();
}

void loop(){
  getPS4buttons();
  settirespeed();
  delay(10);
  getmpu6050();
  delay(10);
}

void mpu6050setup(){
  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void getPS4buttons(void){
  if(PS4.isConnected()){
    digitalWrite(ds4_connect_pin, HIGH);
    if (PS4.L2()) L2 = PS4.L2Value();
    else L2 = 0;
    if (PS4.R2()) R2 = PS4.R2Value();
    else R2 = 0;
    if (PS4.Cross()) CrossButton = 1;
    else CrossButton = 0;
    if(PS4.LStickX()) L3X = PS4.LStickX();
    else L3X = 0;
    if(PS4.LStickY()) L3Y = PS4.LStickY();
    else L3Y = 0;
    if(PS4.Circle()) digitalWrite(power_pin, LOW);
    else digitalWrite(power_pin, HIGH);
  }
  else digitalWrite(ds4_connect_pin, LOW);
}

void settirespeed(void){
  left_cmd = int(150 * float(R2)/255.0 + 100 * float(L3X)/128.0);
  right_cmd = int(150 * float(R2)/255.0 - 100 * float(L3X)/128.0);
  if(CrossButton){
    left_cmd *= -1;
    right_cmd *= -1;
  }
  left.set_speed(left_cmd);
  right.set_speed(right_cmd);
  Serial.println(left.SPEED);
  Serial.println(right.SPEED);

}

void getmpu6050(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
}
