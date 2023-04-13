void encoderHandler() {
  W_FR = (float)((FR_enc.read() / (0.01 * ppr)) * (2 * pi));  // Calculate how many 'circles' we've been through
  FR_enc.write(0);
  W_BR = (float)((BR_enc.read() / (0.01 * ppr)) * (2 * pi));
  BR_enc.write(0);
  W_BL = (float)((BL_enc.read() / (0.01 * ppr)) * (2 * pi));
  BL_enc.write(0);
  W_FL = (float)((FL_enc.read() / (0.01 * ppr)) * (2 * pi));
  FL_enc.write(0);

  W_R = (float)((R_enc.read() / (0.01 * 600)) * (2 * pi));
  R_enc.write(0);  //TES
  W_L = (float)((L_enc.read() / (0.01 * 600)) * (2 * pi));
  L_enc.write(0);

  kinematic();  // Calculate the real distance we've been through

  Px += Vx * encoderRate / 1000;
  Py += Vy * encoderRate / 1000;
  theta += w * encoderRate / 1000;
  Px0 += Vx0 * encoderRate / 1000;
  Py0 += Vy0 * encoderRate / 1000;

  vel = sqrt(Vx * Vx + Vy * Vy);
}

void kinematic() {
  Vx = (+0.35355 * W_FR - 0.35355 * W_BR - 0.35355 * W_BL + 0.35355 * W_FL) * wheelRadius * 100;
  Vy = (-0.35355 * W_FR - 0.353551 * W_BR + 0.35355 * W_BL + 0.35355 * W_FL) * wheelRadius * 100;
  w = (+W_FR / 4 + W_BR / 4 + W_BL / 4 + W_FL / 4) * wheelRadius / robotRadius * (180 / pi);

  Vx0 = cos(-theta_bno * pi / 180) * Vx - sin(-theta_bno * pi / 180) * Vy;
  Vy0 = sin(-theta_bno * pi / 180) * Vx + cos(-theta_bno * pi / 180) * Vy;
}

void kinematic_ext() {  //TES
  x_ext = (W_L * 0.70711 + W_R * 0.70711) * 0.03 * 100;
  y_ext = (W_L * -0.70711 + W_R * 0.70711) * 0.03 * 100;
  w_ext = (-W_L / 2 + -W_R / 2) * 0.03 / 0.03 * (180 / pi);
}  //TES
