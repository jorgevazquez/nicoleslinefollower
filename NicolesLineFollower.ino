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
  IN1.run(FORWARD);
  IN4.run(FORWARD);
  Serial.println("go forward!");
}

// Function to go backward
void back(){
  IN1.setSpeed(carSpeed);
  IN4.setSpeed(carSpeed);
  IN1.run(BACKWARD);
  IN4.run(BACKWARD);
  Serial.println("go back!");
}

// Function to go left
void left(){
  IN2.setSpeed(carSpeed);
  IN4.setSpeed(carSpeed);
  IN1.run(RELEASE);
  IN4.run(FORWARD);
  Serial.println("go left!");
}

// Function to go right
void right(){
  IN1.setSpeed(carSpeed);
  IN3.setSpeed(carSpeed);
  IN1.run(FORWARD);
  IN4.run(RELEASE);
  Serial.println("go right!");
}

// Function to stop
void stop(){
  IN1.run(BRAKE);
  IN4.run(BRAKE);
  Serial.println("Stop!");
}

// Function to read the sensors
int readLine(){
  int left,middle,right;
  left = LT_L;
  middle = LT_M;
  right = LT_R;

  Serial.print( "Left is ");
  Serial.println ( left );
  Serial.print( "Middle is ");
  Serial.println ( middle );
  Serial.print( "Right is ");
  Serial.println ( right );
  return left,middle,right;
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
  pinMode(LT_R,INPUT);
  pinMode(LT_M,INPUT);
  pinMode(LT_L,INPUT);
  // Stop the car
  stop();
  // Put the sensor in the middle position
  servo1.write(90);
}

void loop() {
  // Line Follower

  if(LT_M){
    forward();
  }
  else if(LT_R) { 
    right();
    while(LT_R);                             
  }   
  else if(LT_L) {
    left();
    while(LT_L);  
  }
  
  // Separated the obstacle detection routines 
  servo1.write(90);  //setservo position according to scaled value
  delay(500); 
  middleDistance = Distance_test();

  if(middleDistance <= 20) {     
    stop();
    delay(500);                         
    servo1.write(10);          
    delay(1000);      
    rightDistance = Distance_test();
  
    delay(500);
    servo1.write(90);              
    delay(1000);                                                  
    servo1.write(180);              
    delay(1000); 
    leftDistance = Distance_test();
  
    delay(500);
    servo1.write(90);              
    delay(1000);
    if(rightDistance > leftDistance) {
      right();
      delay(360);
    }
    else if(rightDistance < leftDistance) {
    left();
    delay(360);
    }
    else if((rightDistance <= 20) || (leftDistance <= 20)) {
      back();
      delay(180);
    }
  }  
  forward();                     
}
