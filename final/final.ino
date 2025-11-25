#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "odometry.h"
#include "Map.h"
#include "sonar.h"
//#include "PIDcontroller. h"
#include <Gaussian.h>
#include <ArduinoJson.h>
#include <Servo.h>
#include "PDcontroller.h"

// ----------------- Set up ----------------------------------------//
// Constants and Parameters
#define lenOfMap 180
#define WALL_DIST 10     // Desired distance from wall in cm
#define OBSTACLE_THRESH 15 // Threshold for obstacle (cm)
#define BASE_SPEED 120
#define FAST_SPEED 250  // Speed on white line

#define diaL 3.2 //define physical robot parameters
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

// Wall following values
#define minOutput -100
#define maxOutput 100
#define kp 1
#define kd 1
#define base_speed 100

// Hardware objects
Motors motors;
Encoders encoders;
Sonar sonar(4);
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
LineSensors lineSensors;
OLED display;

// Map object
Map myMap;

// Servo
Servo servo;

// State variables
float wallDist;
float distFromWall = WALL_DIST;
int PDout = 0;
bool is_localized = false;
int movement_cycles = 0;
int currentSpeed = BASE_SPEED;

// Odometry tracking
float x = 0, y = 0, theta = 0;
float x_last = 0, y_last = 0, theta_last = 0;
long encCountsLeft = 0, encCountsRight = 0;
long deltaL = 0, deltaR = 0;

//Sensor readings
unsigned int lineDetectionValues[5];

// Mode/state machine
Mode currentMode = MOVE_FORWARD;

// Wall following state
enum WallFollowState { LEFT_WALL, RIGHT_WALL, RETURNING_TO_START };
WallFollowState wallFollowState = LEFT_WALL;

// Coordinate-based turn mapping for left wall following
struct TurnPoint {
  int x;
  int y;
  int turnDegrees;  // 90 or 180
};

const TurnPoint leftWallTurns[] = {
  {5, 1, 90},
  {6, 1, 90},
  {7, 2, 180},
  {4, 2, 90},
  {1, 2, 90},
  {1, 0, 180}
};
const int leftWallTurnsSize = 6;

// wrapPi function
static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

// Setup all hardware and subsystems
void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(180);
  delay(40);
}

//-------------------------- Main control loop --------------------------//
void loop() {
  wallFollowWithTurns();
}

// -------------------------------------------- Core Behaviors --------------------------------------//

void wallFollowWithTurns() {
  // Update odometry
  CalculateDistance();
  
  // Get current cell coordinates
  int current_x = (int)(x / 20.0);  // x in cm / 20cm per cell
  int current_y = (int)(y / 20.0);  // y in cm / 20cm per cell
  
  Serial.print("Position: (");
  Serial.print(current_x);
  Serial.print(",");
  Serial.print(current_y);
  Serial.print(") State: ");
  Serial.println(wallFollowState);
  
  if (wallFollowState == LEFT_WALL) {
    // Check for turns at specific coordinates
    for (int i = 0; i < leftWallTurnsSize; i++) {
      if (current_x == leftWallTurns[i].x && current_y == leftWallTurns[i].y) {
        if (leftWallTurns[i].turnDegrees == 90) {
          turn_left();
        } else if (leftWallTurns[i].turnDegrees == 180) {
          reverse();
        }
        delay(300);
        return;  // Skip wall following this cycle after turn
      }
    }
    
    // Check if we reached the goal
    if (current_x == 2 && current_y == 1) {
      Serial.println("GOAL REACHED! Switching to right wall following...");
      reverse();  // 180 degree turn
      wallFollowState = RIGHT_WALL;
      delay(300);
      return;
    }
    
    // Left wall following
    wallFollowLeft();
    
  } else if (wallFollowState == RIGHT_WALL) {
    // Right wall following to return to start
    wallFollowRight();
    
    // Check if we've returned to start (0,0)
    if (current_x == 0 && current_y == 0) {
      Serial.println("RETURNED TO START!");
      stop();
    }
  }
}

void wallFollowLeft() {
  // Left wall following - sonar on left side
  wallDist = sonar.readDist();
  PDout = PDcontroller.update(wallDist, distFromWall);
  
  int leftCmd  = (int)(BASE_SPEED + PDout);
  int rightCmd = (int)(BASE_SPEED - PDout);
  
  leftCmd  = constrain(leftCmd,  -400, 400);
  rightCmd = constrain(rightCmd, -400, 400);
  
  motors.setSpeeds(leftCmd, rightCmd);
}

void wallFollowRight() {
  // Right wall following - sonar on right side
  // For right wall following, reverse the PD output logic
  wallDist = sonar.readDist();
  PDout = PDcontroller.update(wallDist, distFromWall);
  
  int leftCmd  = (int)(BASE_SPEED - PDout);
  int rightCmd = (int)(BASE_SPEED + PDout);
  
  leftCmd  = constrain(leftCmd,  -400, 400);
  rightCmd = constrain(rightCmd, -400, 400);
  
  motors.setSpeeds(leftCmd, rightCmd);
}

void reverseFullPath() {
    Serial.println("Reversing path to dock...");

    for (int i = PATH_SIZE - 1; i >= 0; i--) {
        Point target = fullPath[i];

        Serial.print("Returning to cell (");
        Serial.print(target.x);
        Serial.print(",");
        Serial.print(target.y);
        Serial.println(")");
        // Move robot to target if desired
        motors.setSpeeds(BASE_SPEED, BASE_SPEED);
        delay(500);
        motors.setSpeeds(0, 0);
        delay(200); // Placeholder for movement
    }
    Serial.println("Reverse of path complete.");
}

// Odometry & distance calculation
void CalculateDistance() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}

void turn_left() {
  Serial.println("TURN_LEFT (90°)");
  float target_theta = wrapPi(theta + (PI / 2.0));  // +90 degrees
  motors.setSpeeds(-BASE_SPEED, BASE_SPEED);
  
  while (abs(wrapPi(theta - target_theta)) > 0.1) {
    CalculateDistance();
    delay(10);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

void turn_right() {
  Serial.println("TURN_RIGHT (90°)");
  float target_theta = wrapPi(theta - (PI / 2.0));  // -90 degrees
  motors.setSpeeds(BASE_SPEED, -BASE_SPEED);
  
  while (abs(wrapPi(theta - target_theta)) > 0.1) {
    CalculateDistance();
    delay(10);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

void reverse() {
  Serial.println("REVERSE (180° turn)");
  // Turn 180 degrees
  float target_theta = wrapPi(theta + PI);
  motors.setSpeeds(-BASE_SPEED, BASE_SPEED);
  
  while (abs(wrapPi(theta - target_theta)) > 0.2) {
    CalculateDistance();
    delay(10);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

void stop() {
  Serial.println("=== STOP - Traversal Complete ===");
  motors.setSpeeds(0, 0);
  
  // Stop forever
  while(true) {
    delay(1000);
  }
}
