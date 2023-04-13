
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

#include <DueTimer.h>

// #include <LiquidCrystal_I2C.h>
// ENCODER

#include <Encoder.h>

#define FR1 22  // front right A
#define FR2 24  // front right B
#define BR1 38  // back right A
#define BR2 40  // back right B
#define BL1 50  // back left A
#define BL2 52  // back left B
#define FL1 46  // front left A
#define FL2 48  // front left B

#define R_ENC1 34
#define R_ENC2 36
#define L_ENC1 47
#define L_ENC2 45

#define encoderRate 10      // (ms)
#define ppr 535             // pulse per rotation // Why 535?
#define wheelRadius 0.0565  // wheel radius (meter)
#define robotRadius 0.25    // robot radius (meter)
#define pi 3.14159

Encoder FR_enc(FR1, FR2);  //(A,B)
Encoder BR_enc(BR1, BR2);
Encoder BL_enc(BL1, BL2);
Encoder FL_enc(FL1, FL2);
Encoder R_enc(R_ENC1, R_ENC2);  //TES
Encoder L_enc(L_ENC1, L_ENC2);  //TES

hd44780_I2Cexp lcd;

// MOTOR

#define en1 23  // Depan kanan
#define motor1LPWM 3
#define motor1RPWM 2

#define en2 25  // Belakang Kanan
#define motor2LPWM 5
#define motor2RPWM 4

#define en3 27  // Belakang Kiri
#define motor3LPWM 7
#define motor3RPWM 6

#define en4 29  // Depan Kiri
#define motor4LPWM 8
#define motor4RPWM 9

// KINEMATIK

#include <DueTimer.h>
#include <pid.h>

#define rRoda 0.0525
#define R 0.24

int maxpwm = 3000;
float maxV = 45.0;  //kecepatan motor saat pwm maksimal
float w1, w2, w3, w4;
int pwm_w1, pwm_w2, pwm_w3, pwm_w4;
float w1_target, w2_target, w3_target, w4_target;
char c;
float x = 0;
float y = 0;
float z = 0;
bool lrotate = 0;
bool rrotate = 0;

double setPoint1, input1;
double setPoint2, input2;
double setPoint3, input3;
double setPoint4, input4;
double Kp1 = 0.5, Ki1 = 0.3, Kd1 = 0;
double Kp2 = 0.5, Ki2 = 0.3, Kd2 = 0;
double Kp3 = 0.5, Ki3 = 0.3, Kd3 = 0;
double Kp4 = 0.5, Ki4 = 0.3, Kd4 = 0;

epid_t PID_W1;
epid_t PID_W2;
epid_t PID_W3;
epid_t PID_W4;

float W_FR, W_BR, W_BL, W_FL;
float W_R, W_L;  //TES
float Vx = 0, Vy = 0, w = 0, Vx0 = 0, Vy0 = 0;
float x_ext = 0, y_ext = 0, w_ext = 0;  //TES
float Px = 0, Py = 0, theta = 0, Px0 = 0, Py0 = 0;
float theta_ext = 0;  //TES
float theta_bno, theta_bno_raw, theta_bno_cal = 0;
float vel;

//

union data {
  struct parameter {
    float w1;
    float w2;
    float w3;
    float w4;
    float x;
    float y;
    float theta;
    float vel;
  } parameter;
  byte packet[32];
} data;

void setup() {
  lcd.begin(20, 4);
  lcd.backlight();
  Serial.begin(115200);
  Serial1.begin(9600);


  Timer4.attachInterrupt(encoderHandler).start(encoderRate * 1000);  // Call encoderHandler every 10 ms
  PID_setup();
  motor_setup();
  kinematik(0, 0, 0);
}

void loop() {
  //  readFromEncoder();
  if (Serial1.available()) {
    c = Serial1.read();
  }
  if (c == 'S') {
    if (!lrotate && !rrotate) {
      x = 0;
      y = 0;
      z = 0;
    } else if (lrotate) {
      z = -0.5;
    } else if (rrotate) {
      z = 0.5;
    }
  } else if (c == 'F') {
    x = 0;
    y = 0.5;
  } else if (c == 'R') {
    x = 0.5;
    y = 0;
  } else if (c == 'B') {
    x = 0;
    y = -0.5;
  } else if (c == 'L') {
    x = -0.5;
    y = 0;
  } else if (c == 'I') {
    x = 0.5;
    y = 0.5;
  } else if (c == 'J') {
    x = 0.5;
    y = -0.5;
  } else if (c == 'H') {
    x = -0.5;
    y = -0.5;
  } else if (c == 'G') {
    x = -0.5;
    y = 0.5;
  } else if (c == 'D') {
    x = 0;
    y = 0;
  } else if (c == 'U') {
    x = 0;
    y = 0;
    lrotate = 1;
  } else if (c == 'u') {
    x = 0;
    y = 0;
    lrotate = 0;
  } else if (c == 'W') {
    x = 0;
    y = 0;
    rrotate = 1;
  } else if (c == 'w') {
    x = 0;
    y = 0;
    rrotate = 0;
  }
  kinematik(0, 0, 0);
  // Serial.print(Vx);
  // Serial.print("  ");
  // Serial.print(Vy);
  // Serial.print("  ");
  // Serial.print(w);
  // Serial.print("  ");
  // Serial.print(Vx_ext);
  // Serial.print("  ");
  // Serial.print(Vy_ext);
  // Serial.print("  ");
  // Serial.print(w_ext);
  // Serial.print("  ");
  Serial.print(W_R);
  Serial.print("  ");
  Serial.println(W_L);
  PID_compute();
  PID_setMotor();
  // setMotor(255, 255, -255, -255) ;
  // Serial.print(c);
  // Serial.print(" 1 : ");
  // Serial.print(x);
  // Serial.print(" 2 : ");
  // Serial.println(y);




  lcd.setCursor(0, 0);
  lcd.print(W_FR);
  lcd.setCursor(10, 0);
  lcd.print(W_BR);
  ;
  lcd.setCursor(0, 1);
  lcd.print(W_BL);
  lcd.setCursor(10, 1);
  lcd.print(W_FL);
  // lcd.clear();
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
