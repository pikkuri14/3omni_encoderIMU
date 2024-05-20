// Define encoder pins
const int encoder1PinA = 2;
const int encoder1PinB = 3;
const int encoder2PinA = 4;
const int encoder2PinB = 5;
const int encoder3PinA = 6;
const int encoder3PinB = 7;


// Wheel 1 (Front Left)
// Wheel 2 (Front Right)
// Wheel 3 (Rear)

// Wheel separation parameters
const float wheelSeparationWidth = 0.3; // meters (distance between left and right wheels)
const float wheelSeparationLength = 0.3; // meters (distance between front and rear wheels)
const float ticksPerMeter = 1000.0; // Example value, adjust to your encoder's specifications

// Odometry variables
volatile long ticks1 = 0;
volatile long ticks2 = 0;
volatile long ticks3 = 0;

float x = 0.0;
float y = 0.0;
float theta = 0.0;

unsigned long lastTime;

void setup() {
  Serial.begin(9600);

  // Set up encoder pins
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(encoder3PinA, INPUT_PULLUP);
  pinMode(encoder3PinB, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinB), updateEncoder3, CHANGE);

  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
  lastTime = currentTime;

  // Calculate wheel travels
  noInterrupts(); // Disable interrupts while reading shared variables
  long localTicks1 = ticks1;
  long localTicks2 = ticks2;
  long localTicks3 = ticks3;
  ticks1 = 0;
  ticks2 = 0;
  ticks3 = 0;
  interrupts(); // Re-enable interrupts

  float travel1 = localTicks1 / ticksPerMeter;
  float travel2 = localTicks2 / ticksPerMeter;
  float travel3 = localTicks3 / ticksPerMeter;

  // Calculate robot movement based on wheel travels
  float deltaXTravel = (travel1 + travel2 + travel3) / 3.0;
  float deltaYTravel = (-travel1 + travel2) / 2.0;
  float deltaTheta = (-travel1 + travel2 - travel3) / (wheelSeparationWidth + wheelSeparationLength);

  // Update robot position
  x += deltaXTravel * cos(theta) - deltaYTravel * sin(theta);
  y += deltaYTravel * cos(theta) + deltaXTravel * sin(theta);
  theta += deltaTheta;

  // Print the updated position
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(theta);

  delay(100); // Adjust as needed
}

void updateEncoder1() {
  static int8_t lastState = 0;
  int8_t newState = (digitalRead(encoder1PinA) << 1) | digitalRead(encoder1PinB);
  ticks1 += transitionTable[lastState][newState];
  lastState = newState;
}

void updateEncoder2() {
  static int8_t lastState = 0;
  int8_t newState = (digitalRead(encoder2PinA) << 1) | digitalRead(encoder2PinB);
  ticks2 += transitionTable[lastState][newState];
  lastState = newState;
}

void updateEncoder3() {
  static int8_t lastState = 0;
  int8_t newState = (digitalRead(encoder3PinA) << 1) | digitalRead(encoder3PinB);
  ticks3 += transitionTable[lastState][newState];
  lastState = newState;
}

const int8_t transitionTable[4][4] = {
  {0, -1, 1, 0},
  {1, 0, 0, -1},
  {-1, 0, 0, 1},
  {0, 1, -1, 0}
};
