#define shoot_pin 42  // Penendang

#define shoot_sens_L 39
#define shoot_sens_R 37

void shoot_ball() {
  digitalWrite(shoot_pin, HIGH);
  delay(100);
  digitalWrite(shoot_pin, LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(shoot_pin, OUTPUT);
  digitalWrite(shoot_pin, LOW);

  pinMode(shoot_sens_L, INPUT);
  pinMode(shoot_sens_R, INPUT);
}

void loop() {
  char msg = Serial.read();

  if (msg == 's') {
    shoot_ball();
  }

  Serial.print(digitalRead(shoot_sens_L));
  Serial.print("  ");
  Serial.println(digitalRead(shoot_sens_R));

  delay(200);
}
