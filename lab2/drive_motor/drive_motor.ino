#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 5
#define PIN_MOTOR_RIGHT_2 6
#define PIN_LED1 A3
#define PIN_LED2 A2

void setup() {
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_2, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_MOTOR_LEFT_1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_2, HIGH);
  delay(1000);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_MOTOR_LEFT_1, HIGH);
  delay(1000);
}

