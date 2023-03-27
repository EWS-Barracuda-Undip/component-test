void PID_setup() {
  epid_info_t epid_err_1 = epid_init(&PID_W1, 0.0, 0.0, 0.0, Kp1, Ki1, Kd1);
  epid_info_t epid_err_2 = epid_init(&PID_W2, 0.0, 0.0, 0.0, Kp2, Ki2, Kd2);
  epid_info_t epid_err_3 = epid_init(&PID_W3, 0.0, 0.0, 0.0, Kp3, Ki3, Kd3);
  epid_info_t epid_err_4 = epid_init(&PID_W4, 0.0, 0.0, 0.0, Kp4, Ki4, Kd4);

  setPoint1 = 0;
  setPoint2 = 0;
  setPoint3 = 0;
  setPoint4 = 0;

  if ((epid_err_1 != EPID_ERR_NONE) && (epid_err_2 != EPID_ERR_NONE) && (epid_err_3 != EPID_ERR_NONE) && (epid_err_4 != EPID_ERR_NONE)) {
    Serial.print("PID Error");
    while (1) {
      ;
    }
  }
}

void PID_reset() {
  PID_setup();
  kinematik(0, 0, 0);
}

void PID_setTarget(float target1, float target2, float target3, float target4) {
  setPoint1 = mapF(target1, -maxV, maxV, -1, 1);
  setPoint2 = mapF(target2, -maxV, maxV, -1, 1);
  setPoint3 = mapF(target3, -maxV, maxV, -1, 1);
  setPoint4 = mapF(target4, -maxV, maxV, -1, 1);
}

void PID_compute() {
  input1 = mapF(W_FR, -maxV, maxV, -1, 1);
  input2 = mapF(W_BR, -maxV, maxV, -1, 1);
  input3 = mapF(W_BL, -maxV, maxV, -1, 1);
  input4 = mapF(W_FL, -maxV, maxV, -1, 1);

  epid_pid_calc(&PID_W1, setPoint1, input1);
  epid_pid_calc(&PID_W2, setPoint2, input2);
  epid_pid_calc(&PID_W3, setPoint3, input3);
  epid_pid_calc(&PID_W4, setPoint4, input4);

  epid_pid_sum(&PID_W1, -1, 1);
  epid_pid_sum(&PID_W2, -1, 1);
  epid_pid_sum(&PID_W3, -1, 1);
  epid_pid_sum(&PID_W4, -1, 1);
}

void PID_setMotor() {
  pwm_w1 = mapF(PID_W1.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w2 = mapF(PID_W2.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w3 = mapF(PID_W3.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w4 = mapF(PID_W4.y_out, -1, 1, -maxpwm, maxpwm);

  lcd.setCursor(0,2);
  lcd.print(pwm_w1);
  lcd.setCursor(10,2);
  lcd.print(pwm_w2);
  lcd.setCursor(0,3);
  lcd.print(pwm_w3);
  lcd.setCursor(10,3);
  lcd.print(pwm_w4);

  setMotor(-pwm_w1, -pwm_w2, -pwm_w3, -pwm_w4);
}

void readFromEncoder() {
  if (Serial2.available()) {
    char c = Serial2.read();
    if (c == '@') {
      Serial2.readBytes(data.packet, sizeof(data.packet));
      //printArray(data.packet, sizeof(data.packet));
      //      Serial.print(" "); Serial.print(data.parameter.w1);
      //      Serial.print(" "); Serial.print(data.parameter.w2);
      //      Serial.print(" "); Serial.print(data.parameter.w3);
      //      Serial.print(" "); Serial.println(data.parameter.w4);
      //
      //      Serial.print("     "); Serial.print(data.parameter.x);
      //      Serial.print(" "); Serial.print(data.parameter.y);
      //      Serial.print(" "); Serial.print(data.parameter.theta);
      //      Serial.print(" "); Serial.println(data.parameter.vel);
    }
  }
}
