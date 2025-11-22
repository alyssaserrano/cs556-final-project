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

// ----------------- Set up ----------------------------------------//
// Constants and Parameters
#define WALL_DIST 20     // Desired distance from wall in cm
#define OBSTACLE_THRESH 15 // Threshold for obstacle (cm)
#define BASE_SPEED 120
#define FAST_SPEED 250  // Speed on white line

// Hardware objects
Motors motors;
Encoders encoders;
Sonar sonar(4);
PIDcontroller pd_obs(0.6, 0.03, 0.25);
Odometry odometry;
LineSensors lineSensors;
OLED display;

// Map object
Map map;

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

//Sensor readings
unsigned int lineSensorValues[5];

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


//-------------------------- Main control loop --------------------------//
// TODO: Implement correct behavior depending on switch case.
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

// -------------------------------------------- Core Behaviors --------------------------------------//

// Update all sensor readings and global variables
void updateSensors() {
  // TODO???: Example: update IR, sonar, bumper readings, etc.
}

//
void atCorner() {
  motors.setSpeeds(0, 0);  // Stop
  delay(500);
  
  turnBody();  // Call your existing turnBody function
  
  currentMode = WALL_FOLLOW;  // Resume wall following
}

void turnBody() {
  // TODO: Turn the body accordingly when we hit a corner or obstacle.

  float start_theta = theta;
  float target_theta = start_theta - (PI / 2.0);  // -90 degrees
  
  // Turn in place (left forward, right backward)
  motors.setSpeeds(BASE_SPEED, -BASE_SPEED);
  
  while (abs(theta - target_theta) > 0.1) {
    CalculateDistance();  // Update odometry
    delay(10);
  }
  
  motors.setSpeeds(0, 0);
  delay(200);  
}


// Run localization step (particle filter, odometry, etc.)
// TODO: Sanity check localization through simulation of some sort.
void updateLocalization() {
  // Localization update
  float dx = x - x_last;
  float dy = y - y_last;
  float dtheta = wrapPi(theta - theta_last);
  particle.move_particles(dx, dy,  dtheta);

  //Measaure, estimation, and resample
  particle.measure();
  particle.print_particles();

  // Mark visited state in map
  map.visited((int)x, (int)y);

  // Check for goals and docking station
  if (map.atGoalLocation((int)x, (int)y)) {
    // Handle goal logic (e.g., set mode, log, etc.)
  }
  if (map.atDockingStation((int)x, (int)y)) {
    // Handle docking logic (e.g., stop, log, etc.)
  }

  //save last odometer reading
  x_last = x;
  y_last = y;
  theta_last = theta;
}

// -------------------------------------Past Lab Functions just in case. -------------------------------------//
void wallFollow() {
  wallDist = sonar.readDist();

  if (wallDist <= 0) {
    // Sensor failed: slow forward
    motors.setSpeeds(BASE_SPEED, BASE_SPEED);
    return;
  }

  if (detectWhiteLine()) {
    currentSpeed = FAST_SPEED;  // Speed up!
  } else {
    currentSpeed = BASE_SPEED;  // Normal speed
  }

  PDout = (int)pd_obs.update(wallDist, distFromWall); // PD controller for distance

  int16_t left = constrain(BASE_SPEED - PDout, -400, 400);
  int16_t right = constrain(BASE_SPEED + PDout, -400, 400);

  motors.setSpeeds(left, right);

  int current_x = (int)(x / 20.0);  // x in cm / 20cm per cell
  int current_y = (int)(y / 20.0);  // y in cm / 20cm per cell
  
  if (map.cornerDetected(current_x, current_y)) {
    currentMode = AT_CORNER;
  }

  // Check for goal arrival
  if (atGoalLocation()) {
    currentMode = AT_GOAL;
  }
}

bool detectWhiteLine(){
  lineSensor.read(lineSensorValues);

  const int whiteThreshold = 1500;
  for (int i = 0; i < 5; i++) {
    if (lineSensorValues[i] < WHITE_THRESHOLD) {
      whiteCount++;
    }
  }
  return (whiteCount >= 2);
}

/*B1 and B2 Code
bool detectBlackSquare(){
  lineSensor.read(lineSensorValues);

  const int blackThreshold = 1500;

  for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] > blackThreshold) {
      blackCount++;
    }
  }
  return (blackCount >= 2);
}

void collectBin() {
  Serial.println("BIN DETECTED!");
  
  // Stop
  motors.setSpeeds(0, 0);
  delay(500);
  
  // Spin 360
  spin360();
  
  // Update counter
  binCount++;
  
  // Flash LED
  for (int i = 0; i < 3; i++) {
    ledYellow(1);
    delay(100);
    ledYellow(0);
    delay(100);
  }
  
  // Beep
  buzzer.playFrequency(880, 200, 15);
  
  Serial.print("Collected bin #");
  Serial.println(binCount);
  
  delay(500);
}

// ===== 360 SPIN =====
void spin360() {
  display.clear();
  display.print("Spin!");
  
  // Time for 360 degree rotation
  // Wheelbase = 9.7cm, circumference = pi * 9.7 = 30.5cm
  // At speed 120, roughly 3-4 seconds for 360
  motors.setSpeeds(120, -120);
  delay(3200);  // Adjust this based on testing
  
  motors.setSpeeds(0, 0);
  delay(300);
}*/

// Odometry & distance calculation
void CalculateDistance() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}
