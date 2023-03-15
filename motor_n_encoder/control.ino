void PID_setMotor() {
  pwm_w1 = mapF(PID_W1.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w2 = mapF(PID_W2.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w3 = mapF(PID_W3.y_out, -1, 1, -maxpwm, maxpwm);
  pwm_w4 = mapF(PID_W4.y_out, -1, 1, -maxpwm, maxpwm);

  //  lcd.clear();
  //  lcd.print((String)pwm_w1 + " " + (String)pwm_w2);
  //  lcd.setCursor(0, 1);
  //  lcd.print((String)pwm_w3 + " " + (String)pwm_w4);

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
