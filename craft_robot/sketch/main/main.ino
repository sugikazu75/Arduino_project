#include <PS4Controller.h>
#include <ESP32Servo.h>

const int power_pin = 2;
const int ds4_connect_pin = 4;
const int stairing_servo_pin = 17;
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

l298n_pwd left(EN_left, IN_PLUS_left, IN_MINUS_left);
l298n_pwd right(EN_right, IN_PLUS_right, IN_MINUS_right);
Servo stairing_servo;

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

  // ========stairing servo===========
  stairing_servo.attach(stairing_servo_pin);
}

void loop(){
  getPS4buttons();
  settirespeed();
  stairing();
  delay(10);

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
    // if(PS4.LStickY()) L3Y = PS4.LStickY();
    // else L3Y = 0;
    if(PS4.Circle()) digitalWrite(power_pin, LOW);
    else digitalWrite(power_pin, HIGH);
  }
  else digitalWrite(ds4_connect_pin, LOW);
}

void settirespeed(void){
  left_cmd = int(R2);
  right_cmd = int(R2);
  if(CrossButton){
    left_cmd *= -1;
    right_cmd *= -1;
  }
  left.set_speed(left_cmd);
  right.set_speed(right_cmd);
  // Serial.println(left.SPEED);
  // Serial.println(right.SPEED);
}

void stairing(void){
  int stairing_servo_cmd = int(-25.0 * L3X / 128.0 + 105.0);
  stairing_servo.write(stairing_servo_cmd);    
}