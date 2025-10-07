/*
  WallFollowingRobotPID.ino
  An autonomous right-wall following robot using PID control with three ultrasonic sensors. The robot moves forward while maintaining a safe distance from the right wall, adjusting its path if the wall is too close or too far.
*/

#include <NewPing.h>

// Motor Pins
#define PWMA 6   // Left Motor Speed pin (ENA)
#define AIN2 A0  // Motor-L forward (IN2)
#define AIN1 A1  // Motor-L backward (IN1)
#define PWMB 5   // Right Motor Speed pin (ENB)
#define BIN1 A2  // Motor-R forward (IN3)
#define BIN2 A3  // Motor-R backward (IN4)

// Ultrasonic Sensor Configuration
#define ECHO_FRONT 2
#define TRIG_FRONT 3
#define ECHO_LEFT 8
#define TRIG_LEFT 7
#define ECHO_RIGHT 10
#define TRIG_RIGHT 9

const int MAX_DISTANCE = 400;
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);
NewPing sonarRight(TRIG_RIGHT, ECHO_RIGHT, MAX_DISTANCE);

// Distance Variables
int distanceFront = 0;
int distanceLeft = 0;
int distanceRight = 0;

// PID Control Variables
int setpoint = 10;  // Desired distance from right wall (cm)
int last_error = 0;
long integral = 0;

// PID Constants (tune these for the robot)
float Kp = 2.0;   // Proportional gain
float Ki = 0.01;  // Integral gain
float Kd = 30.0;  // Derivative gain

// Motor Control
const int base_speed = 55;  // Base forward speed
const int max_speed = 65;
const int min_speed = 50;

// Safety thresholds
int safe_distance_front = 20;
int max_wall_distance = 50;  // If right sensor reads beyond this, no wall detected, for right turn

// Move forward with differential speeds for steering
void forward(int speedLeft, int speedRight) {
  // Constrain speeds
  speedLeft = constrain(speedLeft, 0, max_speed);
  speedRight = constrain(speedRight, 0, max_speed);

  analogWrite(PWMA, speedLeft);
  analogWrite(PWMB, speedRight);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// Turn left in place
void turnLeftInPlace(int speed = 60) {
  Serial.println("Turning Left in place");
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// Turn right in place
void turnRightInPlace(int speed = 60) {
  Serial.println("Turning Right in place");
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// Stop motors
void stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setup() {
  Serial.begin(9600);

  // Set ultrasonic pins
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);

  // Set motor pins as outputs
  pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initial stop
  stop();

  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.println(" cm");
  delay(2000);
}

void loop() {
  // Read distance values (in cm) from each ultrasonic sensor
  distanceFront = sonarFront.ping_cm();
  distanceLeft = sonarLeft.ping_cm();
  distanceRight = sonarRight.ping_cm();

  // Print sensor readings for debugging
  Serial.print("Left: ");
  Serial.println(distanceLeft);
  Serial.print("Front: ");
  Serial.println(distanceFront);
  Serial.print("Right: ");
  Serial.println(distanceRight);

  // Wall in front - turn left
  if ((distanceFront < safe_distance_front) && (distanceFront != 0)) {
    Serial.println(" front wall");
    turnLeftInPlace();
    integral = 0;
    last_error = 0;
    return;
  }

  // No wall on right - turn right to find it
  if ((distanceRight == 0) || (distanceRight > max_wall_distance)) {
    Serial.println(" No wall on right, turn right to search");
    forward(base_speed + 30, base_speed - 20);  // Turn Right
    integral = 0;
    last_error = 0;
    return;
  }

  // PID wall following
  // Calculate error (positive = too far, negative = too close)
  int error = distanceRight - setpoint;

  // Calculate PID terms
  int proportional = error;
  integral += error;
  int derivative = error - last_error;

  // Calculate PID output
  float pid_output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

  // Calculate motor speeds
  // Positive error (too far) -> turn right (slow down right motor)
  // Negative error (too close) -> turn left (slow down left motor)
  int speedLeft = base_speed + pid_output;
  int speedRight = base_speed - pid_output;

  // Ensure minimum speed for both motors
  speedLeft = constrain(speedLeft, min_speed, max_speed);
  speedRight = constrain(speedRight, min_speed, max_speed);

  // Apply motor speeds
  forward(speedLeft, speedRight);

  // Remember last error
  last_error = error;

  // Debug output
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" PID: ");
  Serial.print(pid_output);
  Serial.print(" Left Speed: ");
  Serial.print(speedLeft);
  Serial.print(" Right Speed: ");
  Serial.println(speedRight);
}