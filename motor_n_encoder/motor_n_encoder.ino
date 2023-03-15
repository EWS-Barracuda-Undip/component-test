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

#define encoderRate 10       // (ms)
#define ppr 535                   // pulse per rotation // Why 535?
#define wheelRadius 0.0565 // wheel radius (meter)
#define robotRadius 0.25     // robot radius (meter)
#define pi 3.14159

Encoder FR_enc(FR1, FR2);  //(A,B)
Encoder BR_enc(BR1, BR2);
Encoder BL_enc(BL1, BL2);
Encoder FL_enc(FL1, FL2);

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
#define motor4LPWM 9
#define motor4RPWM 8

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
float Vx = 0, Vy = 0, w = 0, Vx0 = 0, Vy0 = 0;
float Px = 0, Py = 0, theta = 0, Px0 = 0, Py0 = 0;
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
  Serial.begin(9600);

  Timer.getAvailable().attachInterrupt(encoderHandler).start(encoderRate * 1000);  // Call encoderHandler every 10 ms
  kinematik(0, 0, 0);

  setMotor(-1000, -1000, 1000, 1000);
}

void loop() {
  //  readFromEncoder();
  //
  //  PID_compute();
  //  PID_setMotor();
}

float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}