// Include the Servo library
#include <Servo.h>
// And the AFMotor library
#include <AFMotor.h>

// Define object 'servo1' to control the sensor
Servo servo1;

// The pin CONSTANTS for the line tracking sensors
// REPLACE THE NUMBERS IN PARENTHESIS WITH THE ACTUAL
// PIN NUMBERS WHERE YOUR SENSORS ARE CONNECTED!
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)

// DC Motor on M1
AF_DCMotor IN1(1);
// DC Motor on M2
AF_DCMotor IN2(2);
// DC Motor on M3
AF_DCMotor IN3(3);
// DC Motor on M4
AF_DCMotor IN4(4);

// IT IS ASSUMED THAT IN1 is right rear
// IN2 right front, IN3 left rear And
// IN4 left front.  If not, rewire your car
// or shuffle the numbers in PARENTHESIS above


// A constant for our car speed - try slower speeds
// if the car is not able to follow the line quick enough
#define carSpeed 100

// Define variables to store the value of the sensor readings
// A4 and A5 are the ANALOG PINS on the Arduino labeled 4 and 5
int Echo = A4;
int Trig = A5;

// Define variables to hold our right, left, and middle distance values
int rightDistance = 0, leftDistance = 0, middleDistance = 0;

// Function to go forward
void forward(){
  IN1.setSpeed(carSpeed);
  IN4.setSpeed(carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go forward!");
}

// Function to go backward
void back(){
  IN2.setSpeed(carSpeed);
  IN3.setSpeed(carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go back!");
}

// Function to go left
void left(){
  IN2.setSpeed(carSpeed);
  IN4.setSpeed(carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("go left!");
}

// Function to go right
void right(){
  IN1.setSpeed(carSpeed);
  IN3.setSpeed(carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("go right!");
}

// Function to stop
void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stop!");
}

// Function to measure distance with the ultrasonic sensor
int Distance_test() {
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(Trig, LOW);
  float Fdistance = pulseIn(Echo, HIGH);
  Fdistance= Fdistance / 58;
  return (int)Fdistance;
}

// Function to Initialize the Arduino
void setup(){
  // Attach the servo to pin 3
  servo1.attach(3);
  // Start the Serial Monitor
  Serial.begin(9600);
  // Define the mode of our pins
  pinMode(Echo, INPUT);
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LT_R,INPUT);
  pinMode(LT_M,INPUT);
  pinMode(LT_L,INPUT);
  // Stop the car
  stop();
  // Put the sensor in the middle position
  servo1.write(90);
}

void loop() {
  // This is where the loop begins

  // LINE FOLLOWER
  // If we detect the middle sensor
  if(LT_M){
    // Then go forward
    forward();
  }
  // Else if we detect the RIGHT sensor
  else if(LT_R) {
    // Then go right
    right();
    // And keep going right until we don't detect anything on the right sensor
    while(LT_R);
  }
  // Else if we detect something on the left sensor
  else if(LT_L) {
    // Then go left
    left();
    // And keep going left until we don't detect anything on the left sensor
    while(LT_L);
  }
  Otherwise, just keep going straight
  else {
      forward();
  }
  // OBJECT AVOIDANCE
  // Take a reading of what is in front of us
  middleDistance = Distance_test();
  // Print the result in the Serial monitor
  Serial.print("Middle Distance = ");
  Serial.println(middleDistance);

  // If the distance to an obstacle is less than 20
  if(middleDistance <= 20) {
    // STOP!
    stop();
    // Wait half a second
    delay(500);
    // Now go in reverse
    back();
    // And set the servo to point right
    servo1.write(0);
    // Wait a second...
    delay(1000);
    // ...and stop.
    stop();
    //  Read the distance to any obstacle that may be on the right
    rightDistance = Distance_test();
    // Print the right distance on the serial monitor
    Serial.print("Right Distance = ");
    Serial.println(rightDistance);
    // Wait half a second
    delay(500);
    // Return the ultrasound sensor to the middle position
    servo1.write(90);
    // Wait another half a second to give the sensor a chance to catch up
    delay(500);
    // And now set the servo to point to the left
    servo1.write(180);
    // Wait another half a second so it can catch up.
    delay(500);
    // Take a reading to any obstacles that may be on the left
    leftDistance = Distance_test();
    // Print the left distance on the Serial monitor
    Serial.print("Left Distance = ");
    Serial.println(leftDistance);
    // Wait half a second
    delay(500);
    // Return the ultrasound sensor to the middle position
    servo1.write(90);
    // And wait a second
    delay(1000);
    // Now, if the obstacle on the left is close to us
    if(rightDistance > leftDistance) {
      // Then let's go right
      right();
      // And wait a third of a second before we continue
      delay(360);
    }
    // Else if the obstacle on the right is closer to us
    else if(rightDistance < leftDistance) {
      // Then let's go left
      left();
      // And wait a third of a second before we continue
      delay(360);
    }
    // Else if the distance to obstacle on either side is less than 20
    else if((rightDistance <= 20) || (leftDistance <= 20)) {
      // Then let's go in reverse
      back();
      // And wait 180 miliseconds before we continue
      delay(180);
    }
  // But if there is nothing in front of us
  else {
    // Then keep going straight
    forward();
  }
  // Break out of the object avoidance code
}
// Go back to the start of the loop
}
