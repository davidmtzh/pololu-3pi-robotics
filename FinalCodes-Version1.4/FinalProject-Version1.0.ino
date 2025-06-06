#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include "odometry.h" //If using odometry, import odometry.h and odometry.cpp
#include "PIDcontroller.h" //Import your PIDcontroller.h and PIDcontroller.cpp from last lab then uncomment
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h" 

using namespace Pololu3piPlus32U4;


LineSensors lineSensors;
Servo servo;
Motors motors;
Encoders encoders;
Sonar sonar(4);


// Calibration
int calibrationSpeed;
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];
bool isOnBlack = false;
bool isOnBlue = false;
extern int countTrash;
int trashdetected = 0;
int a = 180;
int b = 0;


//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//Update kp, kd, and ki based on your testing (First PIDcontroller for angle)
#define minOutputAng -100
#define maxOutputAng 100
#define kpAng 25 //Tune Kp here
#define kdAng 1.5 //Tune Kd here
#define kiAng 0.5 //Tune Ki here
#define clamp_iAng 50 //Tune ki integral clamp here
#define base_speedAng 50

//Update kp, kd, and ki based on your testing (Second PIDcontroller for velocity) (Task 2.3)
#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 15 //Tune Kp here
#define kdVel 0.5 //Tune Kd here
#define kiVel 1.5 //Tune Ki here
#define clamp_iVel 25 //Tune ki integral clamp here
#define base_speedVel 100

//Update kp and kd based on your PDcontroller
#define minOutput -100
#define maxOutput 100
#define kp 5
#define kd 1.5
#define base_speed 100

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING); //Uncomment if using odometry class
PIDcontroller pidcontroller(kpAng, kiAng, kdAng, minOutputAng, maxOutputAng, clamp_iAng); //Uncomment after you import PIDController
//Write your second PIDcontroller object here (Task 2.3)
PIDcontroller pidcontrollerVel(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel);
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);


// Goal distance from wall (cm)
const double distFromWall=8.0;
double wallDist;


//Feel free to use this in your PD/PID controller for target values
// Given goals in cm and radians
const float goal_x = 10;
const float goal_y = 10;
const float goal_theta = 90; // Must put in radians

// Sonar riding for each direction 
float left, right;
float front = 100;

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;

//Lab 7
//Note: Here are some suggested variables to use for your code.
double PIDout_theta, PIDout_distance; //Output variables for your controllers
double angle_to_goal, actual_angle; //Keeping track of angle
double dist_to_goal = 0.0; //Keeping track of robot's distance to goal location
double PDout;


// Task 2 Rotate To Desired angle
bool rotateTo(float targetAngle) {
  float currentAngle = atan2(sin(theta), cos(theta));
  float output = pidcontroller.update(currentAngle, targetAngle);
  motors.setSpeeds(-output, output);

  if (fabs(currentAngle - targetAngle) < 0.05) {
    motors.setSpeeds(0, 0);
    return true; // rotation complete
  }
  return false; // still rotating
}


void calibrateSensors()
{

  // Spin left slowly for a while
  for (int i = 0; i < 50; i++) {
    lineSensors.calibrate();
    motors.setSpeeds(-100, 100); // turn left
    delay(20);
  }

  // Spin right slowly for a while
  for (int i = 0; i < 50; i++) {
    lineSensors.calibrate();
    motors.setSpeeds(100, -100); // turn right
    delay(20);
  }

  motors.setSpeeds(0, 0); // stop after calibration
  delay(500);

  motors.setSpeeds(100, 100); // Gets out of Start position 
  delay(800);

}

void WallFollowing()
{

    // Constantly read if it detecst a black Square
    //detectBlackLine();

    // Constantly read sonar distance and turn left
    servo.write(180);
    wallDist = sonar.readDist();
    Serial.println(wallDist);


    /*
    if (isOnBlack || foundWall)
    {
      Serial.println("Black line found — exiting wall following.");
      motors.setSpeeds(0, 0);
      delay(450);
      break;
    }
    */


    // PDcontroller
    PDout = PDcontroller.update(wallDist, distFromWall);
    double leftSpeed = base_speed + PDout;
    double rightSpeed = base_speed - PDout;

    motors.setSpeeds(leftSpeed,rightSpeed);
  }


void detectBlueLine()
{

  lineSensors.read(lineDetectionValues);
  //Serial.print("Line Detection Value:", lineDetectionValues);

  int lineSensor = 0;
  const int blackThreshold = 1740;  
  const int blueThreshold = 950;
  isOnBlack = false;
  isOnBlue = false;

    //Detect Blue Square
    for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] > blueThreshold && lineDetectionValues[i] < blackThreshold) {

      isOnBlue = true;// TRASH DETECTED!!

      //Rotate 180 
      motors.setSpeeds(110,-110);
      delay(1800);// stops when detected
      //servo.write(0);
      

    }
  }
}


void detectBlackLine()
{

  lineSensors.read(lineDetectionValues);

  //Serial.print("Line Detection Value:", lineDetectionValues);

  int lineSensor = 0;
  const int blackThreshold = 1740;  
  const int blueThreshold = 750;
  isOnBlack = false;
  isOnBlue = false;


  for (int i = 0; i < 5; i++) {
      lineSensor = lineDetectionValues[i]; 
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(lineSensor);
  }

  // If the sensor detects black, robot rotates
  if (trashdetected > 0 ){
    delay (600);

  }

  // Detect Black Square
  for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] > blackThreshold) {

      isOnBlack = true;// TRASH DETECTED!!

      /*  Hardcoed Escape Trash Protocol 

      motors.setSpeeds(0,0);
      delay(1000);// stops when detected

      motors.setSpeeds(100,100);
      delay(1200);

      motors.setSpeeds(150,-150);
      delay(2125);

      isOnBlack = false;
      WallFollowing();

      //motors.setSpeeds(100,100);
      //delay(1200);
      */
      trashdetected++;
          // Debug: Line Detection Sensor Readings

      break;

    }

  }
    //break;
}

  






void escapeTrash(){
if (isOnBlack == true) 
  {
      countTrash++;

      motors.setSpeeds(0,0);
      delay(1000);// stops when detected

      isOnBlack = true;// TRASH DETECTED!!

      motors.setSpeeds(100,100);
      delay(600);

      motors.setSpeeds(150,-150);
      delay(2025);

      isOnBlack = false;
      //wallfollowing();


  }

}

// Function to Scan Wall distancee (Left, right Front) 
void scanSurroundings(float &leftDist, float &frontDist, float &rightDist) {

  motors.setSpeeds(0,0);
  //delay(1000);// stops when detected
  servo.write(90); // front
  delay(250);
  frontDist = sonar.readDist();

  /* Initial Design
  servo.write(180); // left
  delay(250);
  leftDist = sonar.readDist();

  servo.write(90); // front
  delay(250);
  frontDist = sonar.readDist();

  servo.write(0); // right
  delay(250);
  rightDist = sonar.readDist();
  */

  // Return servo to face wall
  servo.write(0);
}


void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(180);

  calibrateSensors();

}

// Robot Initial State and Goal Settings
enum RobotState { IDLE, ROTATING, MOVING };
RobotState state = MOVING;

float targetAngle = PI / 2;
float targetX = 100, targetY = 100;

// Rotation Interval Settings
float lastCheckX = 0;
float lastCheckY = 0;
float checkInterval = 12.0; // cm
float checkInterval2 = 5.0; // cm


void loop() {

  // Constantly checks for Trash
  //detectBlueLine();


  // Odometry Update 
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);


  Serial.print("Actual Angle: "); Serial.println(actual_angle);
  Serial.print("Angle to Goal: "); Serial.println(angle_to_goal);
  Serial.print("Distance to Goal: "); Serial.println(dist_to_goal);

  // Setting to check current distance Traveled
  float dx = x - lastCheckX;
  float dy = y - lastCheckY;
  float distMoved = sqrt(dx*dx + dy*dy);

  

  // Checks if 5cm interval has passed
  if (distMoved >= checkInterval) {
  
  Serial.print("Distance Moved: "); Serial.print(distMoved);
  scanSurroundings(left, front, right);

  // Save or print the data
  Serial.print("Left: "); Serial.print(left);
  Serial.print("  Front: "); Serial.print(front);
  Serial.print("  Right: "); Serial.println(right);

  // Update last checked position
  lastCheckX = x;
  lastCheckY = y;

    // Checks if 7cm interval has passed to check for black square
  if (distMoved >= checkInterval2) {
  
  detectBlackLine();
  
  // Update last checked position
  lastCheckX = x;
  lastCheckY = y;

}
}



  // FSA 
    switch (state) {
    case ROTATING:
      if (rotateTo(targetAngle)) {
        state = MOVING;
      }
      break;

    case MOVING:
      WallFollowing();
      if (front <= 11) 
      {
        
        state = IDLE;
        front = 20;
      }
      if (isOnBlack == true)
      {

        escapeTrash();
        isOnBlack = false ;

        motors.setSpeeds(0, 0);
        scanSurroundings(left, front, right);
        if (front <= 7) 
          {
          
          // rotate to the righ
            motors.setSpeeds(100,-100);
            delay(800);
            motors.setSpeeds(100, 100);
            delay(600);
            front = 20;
          }

      }

      break;

    case IDLE:
      motors.setSpeeds(0, 0);
      Serial.print("Front Wall Distance:");
      Serial.println(wallDist);

      //rotateTo(-120);
      motors.setSpeeds(100,-100);
      delay(400);

      delay(300);
      motors.setSpeeds(100,100);
      delay(350);
      state = MOVING;
      //motors.setSpeeds(100,100);
      //delay(300);


      break;
  }

  delay(50);
  
}




  /*TASK 2.2
  Improve the baseline solution by telling the robot to stop when it gets close 
  enough to the goal.
  Write your code below and comment out when moving to the next task.*/

  /*TASK 2.3
  Improve the solution further by using a second PID controller to control the velocity
  as it goes towards the goal.
  Write your code below.*/


 