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
