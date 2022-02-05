// Arduino入門編㉒ モータードライバを使いDCモーターを制御する(PWM制御)
// https://burariweb.info
#include <ros.h>
#include <geometry_msgs/Twist.h>
// デジタルピンの定義
const int IN_PLUS_left = 3;
const int IN_MINUS_left = 4;
const int IN_PLUS_right = 5;
const int IN_MINUS_right = 6;
const int EN_left = 9;  // PWM制御で使うEN_leftピンをD9に(モーター1のPWM制御ピン)
const int EN_right = 10; // PWM制御で使うEN_rightピンをD10に(モーター2のPWM制御ピン)

ros::NodeHandle nh;


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

void cmdvel_cb(const geometry_msgs::Twist& msg);

ros::Subscriber<geometry_msgs::Twist> cmdvel_sub("cmd_vel", cmdvel_cb);

void cmdvel_sub_setup(){
  nh.subscribe(cmdvel_sub);
}

void cmdvel_cb(const geometry_msgs::Twist& msg){
  float linear_x = msg.linear.x;
  float linear_y = msg.linear.y;
  left.set_speed(150 * linear_x - 100 * linear_y);
  right.set_speed(150 * linear_x + 100 * linear_y);
  Serial.println(left.SPEED);
  Serial.println(right.SPEED);
}


void setup(){
  nh.initNode();
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  cmdvel_sub_setup();
}

void loop(){
 nh.spinOnce();
}
