/*
  LeftWallFollowingRobotPID.ino
  An autonomous left-wall following robot using PID control with 2 ultrasonic sensors. The robot moves forward while maintaining a safe distance from the left wall, adjusting its path if the wall is too close or too far.
*/

#include <NewPing.h>

// Motor Pins
#define PWMA 6   // Right Motor Speed pin (ENA)
#define AIN2 9   // Motor-R forward (IN1)
#define AIN1 10  // Motor-R backward (IN2)
#define PWMB 5   // Left Motor Speed pin (ENB)
#define BIN1 7   // Motor-L forward (IN3)
#define BIN2 8   // Motor-L backward (IN4)

// Ultrasonic Sensor Configuration
#define ECHO_FRONT 2
#define TRIG_FRONT 3
#define ECHO_LEFT 11
#define TRIG_LEFT 12

const int MAX_DISTANCE = 400;
NewPing sonarFront(TRIG_FRONT, ECHO_FRONT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_LEFT, ECHO_LEFT, MAX_DISTANCE);

// Distance Variables
int distanceFront = 0;
int distanceLeft = 0;

// PID Control Variables
int setpoint = 10;  // Desired distance from right wall (cm)
int last_error = 0;
long integral = 0;

// PID Constants (tune these for the robot)
float Kp = 2.0;   // Proportional gain
float Ki = 0.01;  // Integral gain
float Kd = 30.0;  // Derivative gain

// Motor Control
const int base_speed = 110;  // Base forward speed
const int max_speed = 255;
const int min_speed = 95;

// Safety thresholds
int safe_distance_front = 20;
int max_wall_distance = 25;  // If right sensor reads beyond this, no wall detected, for right turn

int flag = 0;

// Move forward with differential speeds for steering
void forward(int speedLeft, int speedRight) {
  // Constrain speeds
  flag = 0;
  speedLeft = constrain(speedLeft, 0, max_speed);
  speedRight = constrain(speedRight, 0, max_speed);

  analogWrite(PWMA, speedRight);
  analogWrite(PWMB, speedLeft);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// Turn left in place
void turnLeftInPlace(int speed = 100) {
  Serial.println("Turning Left in place");
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// Turn right in place
void turnRightInPlace(int speed = 100) {
  Serial.println("Turning Right in place");
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

// Stop motors
void stop() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(500);
}

void setup() {
  Serial.begin(9600);

  // Set ultrasonic pins
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);

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

  // Print sensor readings for debugging
  Serial.print("Left: ");
  Serial.println(distanceLeft);
  Serial.print("Front: ");
  Serial.println(distanceFront);

  // Wall in front - stop & turn right
  if ((distanceFront < safe_distance_front) && (distanceFront != 0)) {
    Serial.println(" front wall");
    if (flag == 0) {
      stop();
      flag = 1;
    }
    turnRightInPlace();
    integral = 0;
    last_error = 0;
    return;
  }

  // No wall on left - turn left to find it
  if ((distanceLeft == 0) || (distanceLeft > max_wall_distance)) {
    Serial.println(" No wall on left, turn left to search");
    forward(80, 200);
    integral = 0;
    last_error = 0;
    return;
  }

  // PID wall following
  // Calculate error (positive = too far, negative = too close)
  int error = distanceLeft - setpoint;

  // Calculate PID terms
  int proportional = error;
  integral += error;
  int derivative = error - last_error;

  // Calculate PID output
  float pid_output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);

  // Calculate motor speeds
  // Positive error (too far) -> turn left (slow down left motor)
  // Negative error (too close) -> turn right (slow down right motor)
  int speedLeft = base_speed - pid_output;
  int speedRight = base_speed + pid_output;

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