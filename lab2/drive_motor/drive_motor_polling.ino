#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2

int left_position = 0;

bool prevLeftEncoderVal = 0;

void setup() {
  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
 
  Serial.begin(9600);
}

int count = 0;
void loop() {
  bool leftEncoderVal = digitalRead(PIN_ENCODER_LEFT_B);
  bool leftEncoderValA = digitalRead(PIN_ENCODER_LEFT_A);

  // Detect a rising edge of the left encoder’s B channel
  if (leftEncoderVal == HIGH and prevLeftEncoderVal == LOW) {
    leftEncoderRisingEdge();
  }

  prevLeftEncoderVal = leftEncoderVal;

  // Print the position every 1000 loops
  if (count % 1000 == 0) {
    Serial.println(left_position);
  }
  count++;
}

void leftEncoderRisingEdge()
{
  if (digitalRead(PIN_ENCODER_LEFT_A) == HIGH){
    left_position += 1;
  } else {
    left_position -= 1;
  }
}

