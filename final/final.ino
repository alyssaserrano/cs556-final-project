#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"
#include "odometry.h"
#include "Map.h"
#include "sonar.h"
#include "PIDcontroller.h"
#include <Gaussian.h>
#include <ArduinoJson.h>

// Constants and Parameters
#define WALL_DIST 20     // Desired distance from wall in cm
#define OBSTACLE_THRESH 15 // Threshold for obstacle (cm)
#define BASE_SPEED 120

// Hardware objects
Pololu3piPlus32U4Motors motors;
Pololu3piPlus32U4Encoders encoders;
SonarSensor sonar; // Replace with your actual sonar class
PIDcontroller pd_obs(0.6, 0.03, 0.25);
Odometry odometry;

// State variables
float wallDist;
float distFromWall = WALL_DIST;
int PDout = 0;
bool is_localized = false;
int movement_cycles = 0;

// Odometry tracking
float x = 0, y = 0, theta = 0;
float x_last = 0, y_last = 0, theta_last = 0;
long encCountsLeft = 0, encCountsRight = 0;
long deltaL = 0, deltaR = 0;

// Mode/state machine
enum Mode { WALL_FOLLOW, OBSTACLE_AVOID, AT_GOAL, AT_CORNER };
// Start at wall following
Mode currentMode = WALL_FOLLOW;

// Setup all hardware and subsystems
void setup() {
  Serial.begin(9600);

  // Turn head to left
  Servo.attach(90);
}

// Main control loop
void loop() {

  // Select Action
  switch(currentMode){
    case WALL_FOLLOW:
      wallFollow();
      break;
    case AT_GOAL:
      handleGoal();
      break;
    case AT_CORNER:
      atCorner();
      break;
    case OBSTACLE_AVOID:
      obstacleAvoid();
      break;
  }

  
}

// ---- Core Behaviors ----

// Wall following logic
void wallFollow() {
  wallDist = sonar.readDist();

  if (wallDist <= 0) {
    // Sensor failed: slow forward
    motors.setSpeeds(BASE_SPEED, BASE_SPEED);
    return;
  }

  PDout = (int)pd_obs.update(wallDist, distFromWall); // PD controller for distance

  int16_t left = constrain(BASE_SPEED - PDout, -400, 400);
  int16_t right = constrain(BASE_SPEED + PDout, -400, 400);

  motors.setSpeeds(left, right);

  // Check for obstacle ahead
  float frontDist = sonar.readDist();
  if (frontDist < OBSTACLE_THRESH) {
    currentMode = OBSTACLE_AVOID;
  }

  // Check for goal arrival
  if (atGoalLocation()) {
    currentMode = AT_GOAL;
  }
}

// ---- Stubs for required functions ----

// Update all sensor readings and global variables
void updateSensors() {
  // Example: update IR, sonar, bumper readings, etc.
}

// Run localization step (particle filter, odometry, etc.)
void updateLocalization() {
  float dx = x - x_last;
  float dy = y - y_last;
  float dtheta = wrapPi(theta - theta_last);
  particle.move_particles(dx, dy,  dtheta);


  //Measaure, estimation, and resample
  //Calculate particle's posterior probabilities, calculate estimated robot's position, and resample
  //TODO: Put code under here 
  // Update measurement & resample
  particle.measure();


  // Display all particle locations and estimated robot location on screen   
  //TODO: Put code under here 
  // Display particle info
  particle.print_particles();
  
    
  //save last odometer reading
  x_last = x;
  y_last = y;
  theta_last = theta;
}

// Obstacle avoidance behavior
void avoidObstacle() {
  motors.setSpeeds(0, 0);
  delay(300);
  motors.setSpeeds(-BASE_SPEED, BASE_SPEED); // turn left
  delay(400);
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(300);
  currentMode = WALL_FOLLOW;
}

// Detect if robot is at goal location
bool atGoalLocation() {
  
}

// Logic to run when a goal is reached
void handleGoal() {
  
}

// Track Coordinates traveled.
void visited(){
  
}

// Odometry & distance calculation
void CalculateDistance() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}
