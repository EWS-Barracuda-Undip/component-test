void kinematik(float Vx, float Vy, float w) {
  w1_target = -(0.7071 * Vy - 0.7071 * Vx - R * w) / rRoda;
  w2_target = -(0.7071 * Vy + 0.7071 * Vx - R * w) / rRoda;
  w3_target = -(-0.7071 * Vy + 0.7071 * Vx - R * w) / rRoda;
  w4_target = -(-0.7071 * Vy - 0.7071 * Vx - R * w) / rRoda;

  PID_setTarget(w1_target, w2_target, w3_target, w4_target);
}

void PID_setTarget(float target1, float target2, float target3, float target4) {
  setPoint1 = mapF(target1, -maxV, maxV, -1, 1);
  setPoint2 = mapF(target2, -maxV, maxV, -1, 1);
  setPoint3 = mapF(target3, -maxV, maxV, -1, 1);
  setPoint4 = mapF(target4, -maxV, maxV, -1, 1);
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int ang2pwm(float ang) {
  return map(ang, -maxV, maxV, -maxpwm, maxpwm);
}
