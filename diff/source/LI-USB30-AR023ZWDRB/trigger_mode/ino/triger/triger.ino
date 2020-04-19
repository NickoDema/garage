int cam1 = 12;
int cam2 = 11;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(cam1, OUTPUT);
  pinMode(cam2, OUTPUT);
}

void loop() {
//  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(cam1, HIGH);
  digitalWrite(cam2, HIGH);
  delayMicroseconds(1);
//  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(cam1, LOW);
  digitalWrite(cam2, LOW);
  delay(50);
}
