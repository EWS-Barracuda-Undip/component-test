#define en5 31  // Penggiring
#define motor5LPWM 11
#define motor5RPWM 10

void motor_setup() {
  pinMode(en5, OUTPUT);
  pinMode(motor5LPWM, OUTPUT);
  pinMode(motor5RPWM, OUTPUT);
}

void setPenggiring(bool active) {
  analogWriteResolution(12);
  analogWrite(motor5LPWM, 0);
  if (active) {
    digitalWrite(en5, HIGH);
    analogWrite(motor5RPWM, 4000);
  } else {
    digitalWrite(en5, LOW);
    analogWrite(motor5RPWM, 0);
  }
}
