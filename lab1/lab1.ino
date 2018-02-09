#define PIN_LED1 A3
#define PIN_LED2 A2
#define PIN_BUTTON A1


void setup() {
  // put your setup code here, to run once:
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_LED1, HIGH);
  delay(1000);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, HIGH);
  delay(1000);
  digitalWrite(PIN_LED2, LOW);
}
