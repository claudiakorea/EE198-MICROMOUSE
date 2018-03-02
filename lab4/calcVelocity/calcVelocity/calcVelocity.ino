#define PIN_ENCODER_LEFT_A 11
#define PIN_ENCODER_LEFT_B 2
#define PIN_ENCODER_RIGHT_A 12
#define PIN_ENCODER_RIGHT_B 3
#define PIN_MOTOR_LEFT_1 9
#define PIN_MOTOR_LEFT_2 10
#define PIN_MOTOR_RIGHT_1 6
#define PIN_MOTOR_RIGHT_2 5

// Mouse physical parameters
const float ENCODER_TICKS_PER_REVOLUTION = 420.0 / 2.0; // blaze it
const float WHEELBASE_DIAMETER = 95.0; // mm
const float WHEEL_DIAMETER = 34.0; // mm
const float VELOCITY_COEFF = WHEEL_DIAMETER * PI / ENCODER_TICKS_PER_REVOLUTION * 1000000.0;

// Encoder helper variables
unsigned long prev_pulse_time_right;
unsigned long prev_pulse_time_left;

// Encoder state variables
long ticks_left = 0; // ticks
long ticks_right = 0; // ticks
double velocity_left = 0; // millimeters/sec
double velocity_right = 0; // millimeters/sec

double velocity_forward;
double velocity_turn;

int count = 0;

void setup() {
  Serial.begin(9600);

  // Encoder setup
  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_ENCODER_LEFT_B, INPUT);
  pinMode(PIN_ENCODER_RIGHT_A, INPUT);
  pinMode(PIN_ENCODER_RIGHT_B, INPUT);
  pinMode(PIN_ENCODER_LEFT_A, INPUT);
  pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
  pinMode(PIN_MOTOR_LEFT_2, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT_2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT_B), leftEncoderRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT_B), rightEncoderRisingEdge, RISING);
}

void loop() {
  checkEncodersZeroVelocity();

  velocity_forward = (velocity_left + velocity_right) / 2;
  velocity_turn = (velocity_right - velocity_left) / WHEELBASE_DIAMETER;

  analogWrite(PIN_MOTOR_LEFT_1, 0);
  analogWrite(PIN_MOTOR_LEFT_2, 0);
  analogWrite(PIN_MOTOR_RIGHT_1, 0);
  analogWrite(PIN_MOTOR_RIGHT_2, 0);
  
  if (count % 1000 == 0) {
    // Print debug info every 1000 loops
    Serial.print(velocity_turn);
    Serial.print(" ");
    Serial.println(velocity_forward / 100.0); // scale the forwards velocity so it's easier to visualize next to the turning velocity
  }
  count++;
}


//////////////////////
// Helper functions //
//////////////////////

void checkEncodersZeroVelocity(void) {
  // Sets the wheel velocity to 0 if we haven't see an edge in a while
  unsigned long curr_time = micros();
  if (curr_time - prev_pulse_time_left > 100000) {
    velocity_left = 0;
  }
  if (curr_time - prev_pulse_time_right > 100000) {
    velocity_right = 0;
  }
}

void leftEncoderRisingEdge(void) {
  unsigned long curr_time = micros();
  //Serial.print("Anant");
  int direction;
  if (digitalRead(PIN_ENCODER_LEFT_A) == HIGH) {
    direction = 1;
  } else {
    direction = -1;
  }

  if (direction * velocity_left < 0) {
    velocity_left = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_left = direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_left);
  }
  ticks_left += direction;

  prev_pulse_time_left = curr_time;
}

void rightEncoderRisingEdge(void) {
  unsigned long curr_time = micros();
  //Serial.print("Sahai");
  int direction;
  if (digitalRead(PIN_ENCODER_RIGHT_A) == HIGH) {
    direction = -1;
  } else {
    direction = 1;
  }

  if (direction * velocity_right < 0) {
    velocity_right = 0;
  } else {
    // Otherwise, convert the period of our pulse in mm/second
    velocity_right = direction * VELOCITY_COEFF / (curr_time - prev_pulse_time_right);
  }
  ticks_right += direction;

  prev_pulse_time_right = curr_time;
}
