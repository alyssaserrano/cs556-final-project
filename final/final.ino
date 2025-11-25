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
#include <Servo.h>

// ----------------- Set up ----------------------------------------//
// Constants and Parameters
#define WALL_DIST 20     // Desired distance from wall in cm
#define OBSTACLE_THRESH 15 // Threshold for obstacle (cm)
#define BASE_SPEED 120
#define FAST_SPEED 250  // Speed on white line

#define diaL 3.2 //define physical robot parameters
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

// Particle filter
#define lenOfMap 60 //60 cm / 3 = 20 cm per grid unit (matches the map).
#define N_particles 20
#define move_noise   0.01   // σ_d^2
#define rotate_noise 0.05   // σ_θ^2
#define ultra_noise  0.10   // σ_s^2

// Hardware objects
Motors motors;
Encoders encoders;
Sonar sonar(4);
PIDcontroller pd_obs(0.6, 0.03, 0.25, -100.0, 100.0, 1.0);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
LineSensors lineSensors;
OLED display;

// Map object
Map myMap;

// Servo
Servo servo;

// Particle filter
ParticleFilter particle(myMap, lenOfMap, N_particles, move_noise, rotate_noise, ultra_noise);

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
// Start at move forward
Mode currentMode = MOVE_FORWARD;

// wrapPi function
static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

// Setup all hardware and subsystems
void setup() {
  Serial.begin(9600);

  // For localization (lab 11 reference)
  while (!Serial) continue;
  delay(1000);

  lineSensors.initFiveSensors();

  // Initialize servo
  servo.attach(5);
  servo.write(90);  // Turn head left

  //seed RNG for diverse particles.
  randomSeed(analogRead(A0));  
 
  float origin[2] = {0.0, 0.0};
  theta = 0.0f;
  float closestDist = myMap.closest_distance(origin, theta);
  Serial.println("Map test distance:");
  Serial.println(closestDist);
  
  Serial.println("Starting autonomous localization...");
  delay(2000);

}

//-------------------------- Main control loop --------------------------//
void loop() {

  // 1) Update localization.
  updateLocalization();

  // 2) Choose action.
  switch(currentMode){
    case MOVE_FORWARD:
      move_forward();
      break;
    case TURN_LEFT:
      turn_left();
      break;
    case TURN_RIGHT:
      turn_right();
      break;
    case REVERSE:
      reverse();
      break;
    case AT_GOAL:
      at_goal();
      break;
    case STOP:
      stop();
      break;
  }

}

// -------------------------------------------- Core Behaviors --------------------------------------//
//
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

/*
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
*/

// Run localization step (particle filter, odometry, etc.)
// Mark the coordinate we are at.
// TODO: Sanity check localization through simulation of some sort.
void updateLocalization() {
  // Calculate the distance.
  CalculateDistance();

  // Localization update
  float dx = x - x_last;
  float dy = y - y_last;
  float dtheta = wrapPi(theta - theta_last);
  particle.move_particles(dx, dy,  dtheta);

  //Measaure, estimation, and resample
  particle.measure();
  //particle.print_particles();

  // Convert physical coordinates to cell indices for the map
  int cell_x = (int)(x / 20.0);
  int cell_y = (int)(y / 20.0);

  // Mark visit
  myMap.visited(cell_x, cell_y);

  // ===== GOAL LOGIC =====
  if (myMap.atGoalLocation(cell_x, cell_y)) {
    currentMode = AT_GOAL;

        // If at the final goal coordinate of (2,1), trigger reversal
    if (cell_x == 2 && cell_y == 1) {
      Serial.println("Final goal reached. Reversing path now.");
      reverseFullPath();  // Call your reversal function
      currentMode = STOP; // stop after reversing
    }
  }
    else {
      Mode turn = myMap.cornerDetected(cell_x, cell_y);
      if (turn == TURN_LEFT)        currentMode = TURN_LEFT;
      else if (turn == TURN_RIGHT)  currentMode = TURN_RIGHT;
      else if (turn == REVERSE)     currentMode = REVERSE;
      else                          currentMode = MOVE_FORWARD;
  }

  //save last odometer reading
  x_last = x;
  y_last = y;
  theta_last = theta;
}

// Odometry & distance calculation
void CalculateDistance() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}

// Movement Primitive
void move_forward() {
  Serial.println("MOVE_FORWARD");
  motors.setSpeeds(BASE_SPEED, BASE_SPEED);
  delay(500);  // Move forward for 500ms
  motors.setSpeeds(0, 0);
  delay(100);
}

void turn_left() {
  Serial.println("TURN_LEFT");
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
  Serial.println("TURN_RIGHT");
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

void at_goal() {
  Serial.println("AT_GOAL - Goal location reached!");
  motors.setSpeeds(0, 0);
  delay(1000);
  
  // Continue traversing
  currentMode = MOVE_FORWARD;
}

void stop() {
  Serial.println("=== STOP - Traversal Complete ===");
  motors.setSpeeds(0, 0);
  
  // Beep to signal completion
  buzzer.playFrequency(880, 500, 15);
  
  // Stop forever
  while(true) {
    delay(1000);
  }
}


// -------------------------------------Past Lab Functions just in case. -------------------------------------//
/*void wallFollow() {
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

  /*
  // Check for black square (pick bin - Phase 3, B1/B2)
  if (detectBlackSquare()) {
    currentMode = AT_GOAL;  // Trigger pick sequence
    return;
  }  
  

  PDout = (int)pd_obs.update(wallDist, distFromWall); // PD controller for distance

  int16_t left = constrain(BASE_SPEED - PDout, -400, 400);
  int16_t right = constrain(BASE_SPEED + PDout, -400, 400);

  motors.setSpeeds(left, right);

  int current_x = (int)(x / 20.0);  // x in cm / 20cm per cell
  int current_y = (int)(y / 20.0);  // y in cm / 20cm per cell
  
  if (myMap.cornerDetected(current_x, current_y)) {
    currentMode = AT_CORNER;
  }

  // Check for goal arrival
  if (myMap.atGoalLocation((int)x, (int)y)) {
    currentMode = AT_GOAL;
  }
}*/

bool detectWhiteLine(){
  lineSensors.read(lineDetectionValues);

  const int whiteThreshold = 1500;
  int whiteCount = 0;

  for (int i = 0; i < 5; i++) {
    if (lineDetectionValues[i] < whiteThreshold) {
      whiteCount++;
    }
  }
  return (whiteCount >= 2);
}

/*B1 and B2 Code
bool detectBlackSquare(){
  lineSensors.read(lineDetectionValues);

  const int blackThreshold = 1500;
  int blackCount = 0;

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
  
  // Time for 360-degree rotation
  // Wheelbase = 9.7cm, circumference = pi * 9.7 = 30.5cm
  // At speed 120, roughly 3-4 seconds for 360
  motors.setSpeeds(120, -120);
  delay(3200);  // Adjust this based on testing
  
  motors.setSpeeds(0, 0);
  delay(300);
}*/
