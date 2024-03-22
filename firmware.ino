#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define BUZZER_PIN 5 // GPIO pin connected to the buzzer

// Servo configuration
const int numServos = 5;
const int servoPins[numServos] = {0, 1, 2, 3, 4}; // Pins connected to PCA9685

// Current position of each servo
int servoPos[numServos] = {90, 90, 90, 90, 90}; // Initialize all servos to 90 degrees

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // Set all servos to home position
  for (int i = 0; i < numServos; i++) {
    pwm.setPWM(servoPins[i], 0, servoToPulse(servoPos[i]));
  }
  
  pinMode(BUZZER_PIN, OUTPUT); // Set buzzer pin as output
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int servoIndex, targetPos;
    sscanf(input.c_str(), "%d %d", &servoIndex, &targetPos);
    
    // Check if servo index is valid
    if (servoIndex >= 0 && servoIndex < numServos) {
      // Move servo to target position
      moveServo(servoIndex, targetPos);
    }
    // Check if the command is for performing pick and place motion
    else if (input == "pick_and_place") {
      performPickAndPlace();
    }
  }
}

// Function to move servo to a specific position
void moveServo(int servoIndex, int targetPos) {
  if (targetPos < 0) {
    targetPos = 0;
  }
  else if (targetPos > 180) {
    targetPos = 180;
  }
  
  int currentPosition = servoPos[servoIndex];
  
  // Incrementally move servo to target position
  for (int pos = currentPosition; pos <= targetPos; pos++) {
    pwm.setPWM(servoPins[servoIndex], 0, servoToPulse(pos));
    delay(15); // Delay between each increment
    // Activate buzzer while the servo is in motion
    digitalWrite(BUZZER_PIN, HIGH);
    delay(10); // Buzzer humming duration
    digitalWrite(BUZZER_PIN, LOW);
  }
  
  // Update current position
  servoPos[servoIndex] = targetPos;
}

// Function to perform a basic pick and place motion
void performPickAndPlace() {
  // Define the motion sequence for pick and place
  int pickSequence[numServos] = {45, 90, 0, 90, 90}; // Example pick position
  int placeSequence[numServos] = {90, 90, 90, 90, 45}; // Example place position
  
  // Execute pick motion
  for (int i = 0; i < numServos; i++) {
    moveServo(i, pickSequence[i]);
  }
  delay(1000); // Add a delay between pick and place
  
  // Execute place motion
  for (int i = 0; i < numServos; i++) {
    moveServo(i, placeSequence[i]);
  }
}

// Function to convert degrees to pulse length
uint16_t servoToPulse(int angle) {
  uint16_t pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}
