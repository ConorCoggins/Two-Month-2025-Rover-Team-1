void setup() {
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
}

void loop() {
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    Serial.println("Blue");
    delay(1000);
    digitalWrite(4, LOW);
    digitalWrite(7, HIGH);
    Serial.println("Yellow");
    delay(1000);
}