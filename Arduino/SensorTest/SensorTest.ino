/*
  Arduino Uno + L298N + HC-SR04 - Improved Obstacle Avoidance Robot
  
  Features:
  - Motors move forward continuously
  - Stops when object detected at <= 10 cm
  - Resumes when object is >= 15 cm (hysteresis prevents oscillation)
  - Distance filtering for more reliable readings
  - Serial debug output
  
  Wiring:
    L298N: ENA->D9, IN1->D3, IN2->D4, ENB->D10, IN3->D5, IN4->D6
    HC-SR04: TRIG->D12, ECHO->D11, VCC->5V, GND->GND
    
  Note: Connect L298N motor power supply (7-12V) and ensure common GND with Arduino
*/

// Motor Driver Pins
const int ENA = 9;
const int IN1 = 3;
const int IN2 = 4;
const int ENB = 10;
const int IN3 = 5;
const int IN4 = 6;

// Ultrasonic Sensor Pins
const int TRIG_PIN = 12;
const int ECHO_PIN = 11;

// Distance Thresholds (with hysteresis)
const int STOP_CM   = 10;   // Stop when obstacle is this close
const int START_CM  = 15;   // Resume when obstacle moves beyond this distance

// Motor Speed (adjust if needed: 150-255)
const int DRIVE_PWM = 200;  // Increase to 220-240 if motors are weak

// State Variable
bool isStopped = false;

/**
 * Read distance from HC-SR04 with averaging filter
 * Returns: distance in cm, or -1 if invalid reading
 */
long readDistanceCm() {
  long sum = 0;
  int validCount = 0;
  
  // Take 3 readings and average them
  for (int i = 0; i < 3; i++) {
    // Send trigger pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    // Read echo pulse (timeout after 25ms = ~4.3m max range)
    unsigned long us = pulseIn(ECHO_PIN, HIGH, 25000UL);
    
    if (us > 0) {
      // Convert microseconds to centimeters (sound travels ~58us per cm)
      long cm = (long)(us / 58UL);
      
      // Validate reading (HC-SR04 range: 2-200 cm)
      if (cm >= 2 && cm <= 200) {
        sum += cm;
        validCount++;
      }
    }
    
    delay(10);  // Small delay between readings
  }
  
  // Return average of valid readings
  return (validCount > 0) ? (sum / validCount) : -1;
}

/**
 * Move both motors forward at specified PWM speed
 */
void moveForward(int pwm) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // Set speed
  analogWrite(ENA, pwm);
  analogWrite(ENB, pwm);
}

/**
 * Stop both motors completely
 */
void stopMotors() {
  // Set PWM to zero
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Set all inputs LOW for brake
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  Serial.println("=== Obstacle Avoidance Robot ===");
  Serial.println("Starting up...");
  
  // Configure motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Start with motors stopped
  stopMotors();
  isStopped = false;
  
  Serial.println("Ready!");
  delay(1000);  // Brief pause before starting
}

void loop() {
  // Read distance from sensor
  long distance = readDistanceCm();
  
  // Hysteresis logic to prevent rapid on/off switching
  if (distance != -1) {
    if (distance <= STOP_CM) {
      // Obstacle too close - stop
      isStopped = true;
    } else if (distance >= START_CM) {
      // Obstacle far enough - resume
      isStopped = false;
    }
    // Between STOP_CM and START_CM: maintain current state
  }
  
  // Control motors based on state
  if (isStopped) {
    stopMotors();
    Serial.print("STOPPED | Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    moveForward(DRIVE_PWM);
    Serial.print("MOVING  | Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }
  
  // Small delay for stability (adjust if needed)
  delay(50);
}
