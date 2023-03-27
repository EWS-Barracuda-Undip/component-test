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

void kinematik(float Vx, float Vy, float w) {
  w1_target = -(0.7071 * Vy - 0.7071 * Vx + R * w) / rRoda;
  w2_target = -(0.7071 * Vy + 0.7071 * Vx + R * w * 0) / rRoda;
  w3_target = -(-0.7071 * Vy + 0.7071 * Vx + R * w) / rRoda;
  w4_target = -(-0.7071 * Vy - 0.7071 * Vx + R * w * 0) / rRoda;

  PID_setTarget(w1_target, w2_target, w3_target, w4_target);
}

int ang2pwm(float ang) {
  return map(ang, -maxV, maxV, -maxpwm, maxpwm);
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
