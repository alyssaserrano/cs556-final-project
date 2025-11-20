//Lab 11 - Autonomous Localization with Particle Filter

#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"
#include "odometry.h"
#include "Map.h"
#include "sonar.h"
#include "PIDcontroller.h"

// Upload libraries
#include <Gaussian.h>
#include <ArduinoJson.h>


// Includes and Definitions
#include <YourSensorLibraries.h>
#define WALL_DIST 20     // Desired distance from wall in cm
#define OBSTACLE_THRESH 15 // Obstacle detected below this distance (cm)

// Setup: sensors, motors, serial
void setup() {
  Serial.begin(9600);
  setupMotors();
  setupSensors();
  // Any initialization code
}

// State
enum Mode { WALL_FOLLOW, OBSTACLE_AVOID, AT_GOAL };
Mode currentMode = WALL_FOLLOW;

void loop() {
  updateSensors(); // Update global sensor/state variables
  
  if (atGoalLocation()) {
    currentMode = AT_GOAL;
    handleGoal();
    // Select next goal or finish
  }
  else if (isObstacleAhead()) {
    currentMode = OBSTACLE_AVOID;
    avoidObstacle();
    // When clear, return to wall following
  }
  else {
    currentMode = WALL_FOLLOW;
    wallFollow();
  }
  // Debug output, optional
  Serial.println(currentMode);
}

// Function stubs
void wallFollow() {
  // Maintain WALL_DIST from wall using sensor readings
  // Steer left or right as needed
    // Keep reading sonar and compute lateral correction
  wallDist = sonar.readDist();
  if (wallDist <= 0) {
    // bad read: just creep forward
    motors.setSpeeds(baseSpeed, baseSpeed);
  } else {
    PDout = (int)pd_obs.update(wallDist, distFromWall); // control effort

    int forward = 120;
    int16_t lSpeed = (int16_t)constrain(forward - PDout, -400, 400);
    int16_t rSpeed = (int16_t)constrain(forward + PDout, -400, 400);
    motors.setSpeeds(lSpeed, rSpeed);
}

bool isObstacleAhead() {
  // Return true if obstacle detected within OBSTACLE_THRESH
  float front_dist = sonar.readDist();

    
  if (front_dist < OBSTACLE_THRESH) {
    return true;
  } else {
  return false;
  }
}

void avoidObstacle() {
  // Example: Stop, turn away from the obstacle, resume wall follow
}

bool atGoalLocation() {
  // Use position tracking or goal markers
}

void handleGoal() {
  // Mark goal as visited, update goals list, or complete mission
}
void localization(){

}