#define shoot_pin 42  // Penendang

void shoot_ball() {
  digitalWrite(shoot_pin, HIGH);
  delay(100);
  digitalWrite(shoot_pin, LOW);
}

void setup() {
  Serial.begin(9600);

  pinMode(shoot_pin, OUTPUT);
  digitalWrite(shoot_pin, LOW);
}

void loop() {
  char msg = Serial.read();

  if (msg == 's') {
    shoot_ball();
  }

  delay(200);
}
