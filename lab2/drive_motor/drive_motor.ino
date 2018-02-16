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
  digitalWrite(PIN_LED2, LOW);
  analogWrite(PIN_MOTOR_LEFT_1, 64);
  analogWrite(PIN_MOTOR_LEFT_2, 0);
  delay(1000);
  digitalWrite(PIN_LED1, LOW);
  digitalWrite(PIN_LED2, HIGH);
  analogWrite(PIN_MOTOR_LEFT_1, 0);
  analogWrite(PIN_MOTOR_LEFT_2, 64);
  delay(1000);
  digitalWrite(PIN_LED1, HIGH);
  digitalWrite(PIN_LED2, HIGH);
  analogWrite(PIN_MOTOR_LEFT_1, 64);
  analogWrite(PIN_MOTOR_LEFT_2, 64);
  delay(1000);
}

