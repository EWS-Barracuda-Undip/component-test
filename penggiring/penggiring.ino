bool ball_reached = false;

void setup() {
  limit_sw_setup();
  motor_setup();
}

void loop() {
  ball_check();
  setPenggiring(ball_reached);
  delay(200);
}
