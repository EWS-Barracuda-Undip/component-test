void motor_setup() {
  pinMode(en1, OUTPUT);
  pinMode(motor1LPWM, OUTPUT);
  pinMode(motor1RPWM, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(motor2LPWM, OUTPUT);
  pinMode(motor2RPWM, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(motor3LPWM, OUTPUT);
  pinMode(motor3RPWM, OUTPUT);
  pinMode(en4, OUTPUT);
  pinMode(motor4LPWM, OUTPUT);
  pinMode(motor4RPWM, OUTPUT);
}

void PID_setMotor() {
  pwm_w1 = mapF(PID_W1.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w2 = mapF(PID_W2.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w3 = mapF(PID_W3.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w4 = mapF(PID_W4.y_out, -1, 1, -maxpwm, maxpwm);

  lcd.setCursor(0, 2);
  lcd.print(pwm_w1);
  lcd.setCursor(10, 2);
  lcd.print(pwm_w2);
  lcd.setCursor(0, 3);
  lcd.print(pwm_w3);
  lcd.setCursor(10, 3);
  lcd.print(pwm_w4);

  setMotor(-pwm_w1, -pwm_w2, -pwm_w3, -pwm_w4);
}

void setMotor(int PWM1, int PWM2, int PWM3, int PWM4) {
  if (PWM1 < -maxpwm) {  //MOTOR1
    PWM1 = -maxpwm;
  } else if (PWM1 > maxpwm) {
    PWM1 = maxpwm;
  }
  if (PWM1 < 0) {
    PWM1 *= -1;
    digitalWrite(en1, HIGH);
    analogWriteResolution(12);
    analogWrite(motor1LPWM, 0);
    analogWrite(motor1RPWM, PWM1);
  } else if (PWM1 > 0) {
    digitalWrite(en1, HIGH);
    analogWriteResolution(12);
    analogWrite(motor1RPWM, 0);
    analogWrite(motor1LPWM, PWM1);
  } else if (PWM1 == 0) {
    digitalWrite(en1, LOW);
    analogWriteResolution(12);
    analogWrite(motor1LPWM, 0);
    analogWrite(motor1RPWM, 0);
  }

  if (PWM2 < -maxpwm) {  //MOTOR2
    PWM2 = -maxpwm;
  } else if (PWM2 > maxpwm) {
    PWM2 = maxpwm;
  }
  if (PWM2 < 0) {
    PWM2 *= -1;
    digitalWrite(en2, HIGH);
    analogWriteResolution(12);
    analogWrite(motor2LPWM, 0);
    analogWrite(motor2RPWM, PWM2);
  } else if (PWM2 > 0) {
    digitalWrite(en2, HIGH);
    analogWriteResolution(12);
    analogWrite(motor2RPWM, 0);
    analogWrite(motor2LPWM, PWM2);
  } else if (PWM2 == 0) {
    digitalWrite(en2, LOW);
    analogWriteResolution(12);
    analogWrite(motor2RPWM, 0);
    analogWrite(motor2LPWM, 0);
  }

  if (PWM3 < -maxpwm) {  //MOTOR3
    PWM3 = -maxpwm;
  } else if (PWM3 > maxpwm) {
    PWM3 = maxpwm;
  }
  if (PWM3 < 0) {
    PWM3 *= -1;
    digitalWrite(en3, HIGH);
    analogWriteResolution(12);
    analogWrite(motor3LPWM, 0);
    analogWrite(motor3RPWM, PWM3);
  } else if (PWM3 > 0) {
    digitalWrite(en3, HIGH);
    analogWriteResolution(12);
    analogWrite(motor3RPWM, 0);
    analogWrite(motor3LPWM, PWM3);
  } else if (PWM3 == 0) {
    digitalWrite(en3, LOW);
    analogWriteResolution(12);
    analogWrite(motor3LPWM, 0);
    analogWrite(motor3RPWM, 0);
  }

  if (PWM4 < -maxpwm) {  //MOTOR4
    PWM4 = -maxpwm;
  } else if (PWM4 > maxpwm) {
    PWM4 = maxpwm;
  }
  if (PWM4 < 0) {
    PWM4 *= -1;
    digitalWrite(en4, HIGH);
    analogWriteResolution(12);
    analogWrite(motor4LPWM, 0);
    analogWrite(motor4RPWM, PWM4);
  } else if (PWM4 > 0) {
    digitalWrite(en4, HIGH);
    analogWriteResolution(12);
    analogWrite(motor4RPWM, 0);
    analogWrite(motor4LPWM, PWM4);
  } else if (PWM4 == 0) {
    digitalWrite(en4, LOW);
    analogWriteResolution(12);
    analogWrite(motor4LPWM, 0);
    analogWrite(motor4RPWM, 0);
  }
}
