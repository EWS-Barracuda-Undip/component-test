#define l_sw_pin 28
#define r_sw_pin 44

void limit_sw_setup() {
  pinMode(l_sw_pin, INPUT_PULLUP);
  pinMode(r_sw_pin, INPUT_PULLUP);
}

void ball_check() {
  bool l_limit = digitalRead(l_sw_pin);
  bool r_limit = digitalRead(r_sw_pin);
  ball_reached = ((!l_limit) && (!r_limit));
  Serial.print(" Left Limit Switch : ");
  Serial.print(!l_limit);
  Serial.print("    Right Limit Switch : ");
  Serial.print(!r_limit);
  Serial.print("    Ball Reached : ");
  Serial.print(ball_reached);
  Serial.println("");
}
