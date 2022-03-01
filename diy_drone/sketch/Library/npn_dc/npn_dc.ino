#include <PS4Controller.h>

int led = 17;           // the pin that the LED is attached to

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  pinMode(led, OUTPUT);
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
    
    analogWrite(led, R2_button);
    Serial.println(R2_button);
//  brightness = brightness + fadeAmount;
//  if (brightness == 0 || brightness == 255) {
//    fadeAmount = -fadeAmount ;
//  }
  delay(10);
}
