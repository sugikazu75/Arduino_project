#include "drone_motor.h"
#include <PS4Controller.h>

//select pwm_pin
const int FR_motor_pin = 2;
const int FL_motor_pin = 17;
const int BR_motor_pin = 26;
const int BL_motor_pin = 32;

drone_motor FR_motor(FR_motor_pin);
drone_motor FL_motor(FL_motor_pin);
drone_motor BR_motor(BR_motor_pin);
drone_motor BL_motor(BL_motor_pin);

void setup(){
    Serial.begin(115200);
    while (!Serial) delay(10);
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

int R2_button=0;

void loop() {
    if (PS4.R2()) R2_button = PS4.R2Value();
    else R2_button = 0;

    FR_motor.set_speed(R2_button);
    FL_motor.set_speed(R2_button);
    BR_motor.set_speed(R2_button);
    BL_motor.set_speed(R2_button);
    
    Serial.println(FR_motor.SPEED);
//  brightness = brightness + fadeAmount;
//  if (brightness == 0 || brightness == 255) {
//    fadeAmount = -fadeAmount ;
//  }
    delay(10);
}
