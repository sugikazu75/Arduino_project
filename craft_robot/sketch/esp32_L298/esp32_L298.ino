#include <PS4Controller.h>

const int IN_PLUS_left = 14;
const int IN_MINUS_left = 27;
const int IN_PLUS_right = 26;
const int IN_MINUS_right = 25;
const int EN_left = 12;
const int EN_right = 33;

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

l298n_pwd left(EN_left, IN_PLUS_left, IN_MINUS_left);
l298n_pwd right(EN_right, IN_PLUS_right, IN_MINUS_right);

int L2=0, R2=0, CrossButton=0, L3X=0, L3Y=0, left_cmd=0, right_cmd=0;

void setup(){
  Serial.begin(115200);
  // =======light setup===========
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(2, HIGH);
  // ==========================
  // =====bluetooth setup=========
  PS4.begin("BC:FE:D9:CF:B6:5D");
  Serial.println("Ready.");
  // =========================
} 

void loop(){
  if(PS4.isConnected()){
    digitalWrite(4, HIGH);
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
    if(PS4.Circle()) digitalWrite(2, LOW);
    else digitalWrite(2, HIGH);
    // Serial.printf("L2 button at %d\n", L2);
    // Serial.printf("R2 button at %d\n", R2);
    // Serial.printf("Cross Button at %d\n", CrossButton);
    // Serial.printf("Left Stick x at %d\n", L3X);
    // Serial.printf("Left Stick y at %d\n", L3Y);
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
  else digitalWrite(4, LOW);
  delay(50);
}
