class drone_motor{
  public:
    int ANALOG_PIN, SPEED;
    drone_motor(int ANALOG_PIN){
      this -> ANALOG_PIN = ANALOG_PIN;
      this -> SPEED = 0;
      pinMode(this -> ANALOG_PIN, OUTPUT);
    }

    void set_speed(int cmd){
      this -> SPEED = cmd;
      analogWrite(this -> ANALOG_PIN, cmd);
    }
};
