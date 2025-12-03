#include <Pololu3piPlus32U4.h>
#include "odometry.h"
#include "sonar.h"
#include <Servo.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

// ====================== CONSTANTS ======================
#define PI 3.14159
#define diaL 3.2
#define diaR 3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

#define minOutput -100
#define maxOutput 100
#define kp 1
#define kd 1
#define BASE_SPEED 100
#define FAST_SPEED 100
#define WALL_DIST 20

// Phase 3/4: IR Sensor Detection
#define BLUE_MIN_CAL 200
#define BLUE_MAX_CAL 500
#define BLACK_THRESHOLD 900

// ====================== HARDWARE OBJECTS ======================
Motors motors;
Encoders encoders;
Sonar sonar(4);
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
LineSensors lineSensors;
OLED display;
Servo servo;

// ====================== STATE VARIABLES ======================
float wallDist;
float distFromWall = WALL_DIST;
int PDout = 0;
int currentSpeed = BASE_SPEED;
long deltaL = 0;
long deltaR = 0;
const float goal_theta;
float theta;

enum Mode { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, AT_GOAL, REVERSE, STOP, WALL_FOLLOW_LEFT, NONE };
Mode currentMode;

// Sensor readings
unsigned int lineDetectionValues[5];

// ====================== SETUP ======================
void setup() {
  Serial.begin(9600);
  delay(2000);
  servo.attach(5);
  servo.write(0);
  delay(40);

  // Calibrate line sensors for Phase 3/4
  Serial.println("Calibrating sensors...");
  calibrateSensors();
  delay(1000);
}

// ====================== MAIN CONTROL LOOP ======================
void loop() {
  Serial.println("=== FORWARD PATH ===");
  
    switch(currentMode){
    case MOVE_FORWARD:
      moveForward(20.0);
      currentMode = WALL_FOLLOW_LEFT;
      break;
    case WALL_FOLLOW_LEFT:
      leftTurn();
      break;
  }

  // Go forward 60 cm
  /*moveForward(20.0);
  //delay(500);

  moveForward(20.0);
  delay(500);

  moveForward(20.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 100 cm
  wallLeft(100.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 40 cm
  wallRight(40.0);
  delay(500);

  // Turn right
  rightTurn();
  delay(500);

  // Go forward 20 cm
  wallRight(20.0);
  delay(500);

  // Turn right
  rightTurn();
  delay(500);

  // Go forward 20 cm
  wallLeft(20.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 20 cm
  wallLeft(20.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 40 cm
  wallLeft(40.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 60 cm
  wallLeft(60.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 20 cm
  wallLeft(20.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 40 cm
  wallRight(40.0);
  delay(500);

  // U-turn
  uTurn();
  delay(500);

  // Go forward 40 cm
  wallLeft(40.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 60 cm
  wallLeft(60.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 40 cm
  wallRight(40.0);
  delay(500);

  // Turn right
  rightTurn();
  delay(500);

  // Go forward 60 cm
  wallRight(60.0);
  delay(500);

  // Turn right
  rightTurn();
  delay(500);

  // Go forward 40 cm
  wallLeft(40.0);
  delay(500);

  // U-turn
  uTurn();
  delay(500);

  // Go forward 20 cm
  wallRight(20.0);
  delay(500);

  // Turn left
  leftTurn();
  delay(500);

  // Go forward 20 cm
  wallLeft(20.0);
  delay(500);

  Serial.println("=== REVERSE PATH ===");
  delay(2000);

  // Go forward 20 cm (reversed: wallRight)
  wallRight(20.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 20 cm (reversed: wallRight)
  wallRight(20.0);
  delay(500);

  // U-turn
  uTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallLeft)
  wallLeft(40.0);
  delay(500);

  // Turn left (reversed: leftTurn)
  leftTurn();
  delay(500);

  // Go forward 60 cm (reversed: wallLeft)
  wallLeft(60.0);
  delay(500);

  // Turn left (reversed: leftTurn)
  leftTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallLeft)
  wallLeft(40.0);
  delay(500);

  // Turn left (reversed: leftTurn)
  leftTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallRight)
  wallRight(40.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 60 cm (reversed: wallRight)
  wallRight(60.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallRight)
  wallRight(40.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallRight)
  wallRight(40.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 20 cm (reversed: wallRight)
  wallRight(20.0);
  delay(500);

  // Turn right (reversed: rightTurn)
  rightTurn();
  delay(500);

  // Go forward 40 cm (reversed: wallLeft)
  wallLeft(40.0);
  delay(500);

  // Turn left (reversed: leftTurn)
  leftTurn();
  delay(500);

  // Go forward 100 cm (reversed: wallLeft)
  wallLeft(100.0);
  delay(500);

  // Turn left (reversed: leftTurn)
  leftTurn();
  delay(500);

  // Go forward 60 cm (reversed: wallLeft)
  wallLeft(60.0);
  delay(500);*/

  Serial.println("=== MISSION COMPLETE!  ===");
  //while(1);
}


// =============== MOVE FORWARD ================================
void moveForward(float targetDist) {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  while(distanceTraveled < targetDist) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders. getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;

    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;
/*
    // ===== PHASE 3/4: CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];

    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD) {
      Serial.println("BLACK SQUARE DETECTED!  Picking...");
      motors.setSpeeds(0, 0);
      delay(200);

      // 360° spin
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);

      // Reset distance tracking
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      leftTotal = 0;
      rightTotal = 0;
      distanceTraveled = 0;
      continue;
    }

    // PHASE 4: Blue line detection (speed up)
    if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
      currentSpeed = FAST_SPEED;
    } else {
      currentSpeed = BASE_SPEED;
    }
*/  
    theta = (deltaR - deltaL) / w;

    PDout = PDcontroller.update(theta, goal_theta);
    motors.setSpeeds(int(currentSpeed), int(currentSpeed));
  }

  motors.setSpeeds(0, 0);
  currentSpeed = BASE_SPEED;
}

// ====================== WALL FOLLOW LEFT ======================
void wallLeft(float targetDist) {
  servo.write(180);

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  while(distanceTraveled < targetDist) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders. getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;

    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;

    wallDist = sonar.readDist();
    Serial.println(wallDist);
/*
    // ===== PHASE 3/4: CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];

    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD) {
      Serial.println("BLACK SQUARE DETECTED!  Picking...");
      motors.setSpeeds(0, 0);
      delay(200);

      // 360° spin
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);

      // Reset distance tracking
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      leftTotal = 0;
      rightTotal = 0;
      distanceTraveled = 0;
      continue;
    }

    // PHASE 4: Blue line detection (speed up)
    if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
      currentSpeed = FAST_SPEED;
    } else {
      currentSpeed = BASE_SPEED;
    }
*/
    // Get PD output
    PDout = PDcontroller.update(wallDist, distFromWall);
    motors.setSpeeds(int(currentSpeed - PDout), int(currentSpeed + PDout));
  }

  //motors.setSpeeds(0, 0);
  //currentSpeed = BASE_SPEED;
}

// ====================== WALL FOLLOW RIGHT ======================
void wallRight(float targetDist) {
  servo.write(0);

  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  while(distanceTraveled < targetDist) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;

    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;

    wallDist = sonar.readDist();
    Serial.println(wallDist);
/*
    // ===== PHASE 3/4: CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];

    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD) {
      Serial.println("BLACK SQUARE DETECTED! Picking...");
      motors.setSpeeds(0, 0);
      delay(200);

      // 360° spin
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);

      // Reset distance tracking
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      leftTotal = 0;
      rightTotal = 0;
      distanceTraveled = 0;
      continue;
    }

    // PHASE 4: Blue line detection (speed up)
    if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
      currentSpeed = FAST_SPEED;
    } else {
      currentSpeed = BASE_SPEED;
    }
*/
    // Get PD output
    PDout = PDcontroller.update(wallDist, distFromWall);
    motors.setSpeeds(int(currentSpeed + PDout), int(currentSpeed - PDout));
  }

  motors.setSpeeds(0, 0);
  currentSpeed = BASE_SPEED;
}

// ====================== TURN FUNCTIONS ======================
void leftTurn() {
  Serial.println("LEFT TURN");
  motors.setSpeeds(50, -50);
  delay(1800);
  motors.setSpeeds(0, 0);
  delay(200);
}

void rightTurn() {
  Serial.println("RIGHT TURN");
  motors. setSpeeds(-50, 50);
  delay(1800);
  motors. setSpeeds(0, 0);
  delay(200);
}

void uTurn() {
  Serial.println("U-TURN");
  motors.setSpeeds(-50, 50);
  delay(3600);
  motors.setSpeeds(0, 0);
  delay(200);
}

// ====================== CALIBRATION ======================
void calibrateSensors() {
  delay(1000);
  Serial.println("Calibrating sensors...");
  
  for(int i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-60, 60);
    } else {
      motors.setSpeeds(60, -60);
    }
    lineSensors.calibrate();
  }
  
  motors.setSpeeds(0, 0);
  Serial.println("Calibration complete!");
}