const int ledPin = 13;
const int buttonRead = A0;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  pinMode(buttonRead, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (digitalRead(buttonRead)) {
    digitalWrite(ledPin, HIGH);
    delay(10);
    digitalWrite(ledPin, LOW);
    delay(40);
  } else {
    digitalWrite(ledPin, LOW);
  }
}
