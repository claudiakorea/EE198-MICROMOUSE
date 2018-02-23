#define PIN_LED1 A3

#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2

int left_position = 0;
double left_velocity = 0;
long last_time = micros();

bool prevLeftEncoderVal = 0;

void setup() {
  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
  pinMode(PIN_LED1, OUTPUT);
 
  Serial.begin(9600);
  // TODO: set up your interrupt here
  // Call function() when pin goes from LOW to HIGH
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), leftEncoderRisingEdge, RISING);
}

void loop() {
  digitalWrite(PIN_LED1, HIGH);
  delay(200);
  digitalWrite(PIN_LED1, LOW);
  delay(200);
  Serial.print("Velocity: ");
  Serial.print(left_velocity);
  Serial.print(" time: ");
  Serial.print(micros());
  Serial.print("\n");
}

void leftEncoderRisingEdge()
{
  if (digitalRead(PIN_ENCODER_LEFT_A) == HIGH){
    left_position += 1;
  } else {
    left_position -= 1;
  }
  long new_time = micros();
  left_velocity = 1000000.0/(new_time - last_time);
  last_time = new_time;
}
