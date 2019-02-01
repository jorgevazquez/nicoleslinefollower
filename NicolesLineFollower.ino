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
#define carSpeed 150

// Define our turn variable
int turn = 0;

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
  IN1.run(RELEASE);
  IN4.run(RELEASE);
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

// Function to measure distance on the left side
int leftDistanceTest() {
    int leftDistance;
    servo1.write(180);
    delay(1000);
    leftDistance = Distance_test();
    Serial.print("Left Distance = ");
    Serial.println(leftDistance);
    servo1.write(90);
    delay(1000);
    return leftDistance;
}

// Function to measure distance on the right side
int rightDistanceTest() {
    int rightDistance;
    servo1.write(0);
    delay(1000);
    rightDistance = Distance_test();
    delay(1000);
    Serial.print("Right Distance = ");
    Serial.println(rightDistance);
    servo1.write(90);
    delay(1000);
    return rightDistance;
}

void avoidObject() {
    // Code to go around an object
    // Which way did we turn?
    switch(turn){
        // We turned left
        case 0:
        // Keep going straight
        forward();
        // Wait a third of a second
        delay(360);
        // Go right
        right();
        // Wait a third of a second
        delay(360);
        // And make a left
        left();
        // wait a third of a second
        delay(360);
        // and go forward
        forward();
        // and get out of this switch
        break;

        // We turned left
        case 3:
        // Keep going straight
        forward();
        // Wait a third of a second
        delay(360);
        // Go left
        left();
        // Wait a third of a second
        delay(360);
        // And make a right
        right();
        // wait a third of a second
        delay(360);
        // and go forward
        forward();
        // and get out of this switch
        break;
        
        default:
        forward();
        break;
    }
}

void lineLost(){
    // Code to exeecute when the line is lost
    // stop!
    stop();
    // Let's wait a second
    delay(360);
    // and go back
    back();
    // wait half a second
    delay(500);
    // and go forward
    forward();
    // wait a third of a second
    delay(360);
    // I will laugh if this is the routine where the car gets stuck
    // Break out of the routine
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

// Now let's run the main program...

void loop() {
  // This is where the loop begins

  // Let's move according to the turn variable
  // Since we set 'turn' in the setup routine then we know 
  // it will have a value of 1 at startup
  switch(turn){
      // If 'turn' is equal to 0 turn right
      case 0:
      right();
      break;
      // If 'turn' is equal to 1 go forward  
      case 1:
      forward();
      break;
      // If 'turn' is equal to 2 turn left
      case 2:
      left();
      break;
      // If 'turn' is equal to 3 go in reverse
      case 4:
      back();
      break;
      // If 'turn' cannot be determined go forward
      default:
      forward();
  }
  // Let's wait half a second before we continue
  delay(500);

  // LINE FOLLOWER
  // Let's read the sensors...
  // ...if we detect the middle sensor
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
    // Else if we have no input from any sensor
    else {
      // We lost our line!
      // lineLost();
      // Keep executing until we find our line
      // while(!LT_M);
      // if we did not acquire our line, let's go straight...
      forward();
      }

  // OBJECT AVOIDANCE
  // Take a reading of what is in front of us  
  middleDistance = Distance_test();
  // Print the result in the Serial monitor
  Serial.print("Middle Distance = ");
  Serial.println(middleDistance);

  // If the distance to an obstacle is less than 30
  if(middleDistance <= 30) {     
    // Don't crash!
    stop();
    // Take a right reading
    rightDistance = rightDistanceTest();
    // Take a left reading
    leftDistance = leftDistanceTest();
    // If the distance to obstacles on either side is less than 20
    if((rightDistance <= 20) || (leftDistance <= 20)) {
      // Then let's go in reverse
      back();
      // And wait two seconds before we continue
      delay(2000);
      // Stop
      stop();
    }
    // Else if the obstacle on the left is closer to us
    if(rightDistance > leftDistance) {
      // Then let's go right
      right();
      // Set the turn variable
      turn = 2;
      // And wait a third of a second before we continue to the next step
      delay(360);
      // Call the object avoidance routine so we can get back on track
      avoidObject();
      // Break out of this conditional
    }
    // Else if the obstacle on the right is closer to us
    else if(rightDistance < leftDistance) {
      // Then let's go left
      left();
      // Set the turn variable
      turn = 0;
      // Wait a third of a second before we continue
      delay(360);
      // And call the object avoidance routine so we can get back on track
      avoidObject();
      // breaK out of this conditional
    }
  // And if there is nothing in front of us
  else {
    // Then keep going straight
    forward();
  }
  // Break out of the object avoidance code
}
// Go back to the start of the loop                     
}
