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

// ====================== CONSTANTS AND PARAMETERS ======================
#define WALL_DIST 10
#define BASE_SPEED 120
#define FAST_SPEED 250

#define diaL 3.2
#define diaR 3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

// Phase 3/4: IR Sensor Detection
#define IR_SENSOR_PIN A0
#define LED_PIN 13
#define BLACK_THRESHOLD 900
#define BLUE_MIN_CAL 200
#define BLUE_MAX_CAL 500
#define TARGET_PICKS 3

// ====================== HARDWARE OBJECTS ======================
Motors motors;
Encoders encoders;
Sonar sonar(4);
PIDcontroller PDcontroller(0.6, 0.03, 0.25, -100.0, 100.0, 1.0);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
LineSensors lineSensors;
OLED display;

// Servo
Servo servo;

// ====================== STATE VARIABLES ======================
float wallDist;
float distFromWall = WALL_DIST;
int PDout = 0;
int currentSpeed = BASE_SPEED;

// Odometry tracking
float x = 0, y = 0, theta = 0;
float x_last = 0, y_last = 0, theta_last = 0;
long encCountsLeft = 0, encCountsRight = 0;
long deltaL = 0, deltaR = 0;

// Sensor readings
unsigned int lineDetectionValues[5];

// ====================== SEGMENT TRACKING ======================
int pickCount = 0;
int executedSegments[50];
int segmentCount = 0;

enum SegmentType {
  WALL_FOLLOW_LEFT,
  WALL_FOLLOW_RIGHT,
  LEFT_TURN,
  RIGHT_TURN,
  U_TURN
};

struct Segment {
  SegmentType type;
  float distance;
};

// Define all forward segments
Segment forwardPath[] = {
  {WALL_FOLLOW_LEFT, 60},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 100},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 40},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 20},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 60},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 40},
  {U_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 60},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 40},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 60},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {U_TURN, 0},
  {WALL_FOLLOW_RIGHT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20}
};

const int NUM_SEGMENTS = 33;

// ====================== SETUP ======================
void setup() {
  Serial.begin(9600);
  while (!Serial) continue;
  delay(1000);

  // Phase 3: IR sensor
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Phase 4: Calibrate line sensors
  Serial.println("Calibrating sensors...");
  calibrateSensors();
  
  Serial.println("Ready to start!");
  delay(2000);
}

// ====================== MAIN CONTROL LOOP ======================
void loop() {
  Serial.println("Starting FORWARD path...");
  
  // ===== FORWARD PATH =====
  for (int i = 0; i < NUM_SEGMENTS && pickCount < TARGET_PICKS; i++) {
    execute_segment(forwardPath[i]);
    executedSegments[segmentCount] = i;
    segmentCount++;
    
    Serial.print("Executed segment ");
    Serial.print(i);
    Serial.print(" | Picks: ");
    Serial.println(pickCount);
  }
  
  // ===== ONCE 3 PICKS FOUND, REVERSE =====
  if (pickCount == TARGET_PICKS) {
    Serial.println("All 3 picks found!      Turning 180...");
    delay(1000);
    uTurn();
    delay(1000);
    
    Serial.println("Starting REVERSE path...");
    
    // Execute stored segments in REVERSE order with SWAPPED turns
    for (int i = segmentCount - 1; i >= 0; i--) {
      Segment seg = forwardPath[executedSegments[i]];
      execute_segment_reversed(seg);
    }
    
    Serial.println("===== MISSION COMPLETE!  =====");
    while(1);
  }
}

// ====================== SEGMENT EXECUTION ======================
void execute_segment(Segment seg) {
  switch (seg.type) {
    case WALL_FOLLOW_LEFT:
      wallFollowLeft(seg.distance);
      break;
    
    case WALL_FOLLOW_RIGHT:
      wallFollowRight(seg.distance);
      break;
    
    case LEFT_TURN:
      leftTurn();
      break;
    
    case RIGHT_TURN:
      rightTurn();
      break;
    
    case U_TURN:
      uTurn();
      break;
  }
}

void execute_segment_reversed(Segment seg) {
  switch (seg.type) {
    case WALL_FOLLOW_LEFT:
      wallFollowRight(seg.distance);
      break;
    
    case WALL_FOLLOW_RIGHT:
      wallFollowLeft(seg.distance);
      break;
    
    case LEFT_TURN:
      rightTurn();
      break;
    
    case RIGHT_TURN:
      leftTurn();
      break;
    
    case U_TURN:
      uTurn();
      break;
  }
}

// ====================== WALL FOLLOWING WITH INTEGRATED DETECTION ======================
void wallFollowLeft(float targetDistance) {
  Serial.print("Wall follow LEFT for ");
  Serial.print(targetDistance);
  Serial.println("cm");
  
  servo.write(180);
  
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;
  
  while(distanceTraveled < targetDistance) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;
    
    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;
    
    wallDist = sonar.readDist();
    
    // ===== CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];
    
    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD && pickCount < TARGET_PICKS) {
      Serial.println("BLACK SQUARE DETECTED!  Picking...");
      motors.setSpeeds(0, 0);
      delay(100);
      
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);
      
      pickCount++;
      display.clear();
      display.print("Picks: ");
      display.print(pickCount);
      flash_led();
      
      delay(500);
      
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      leftTotal = 0;
      rightTotal = 0;
      distanceTraveled = 0;
      continue;
    }
    
    // PHASE 4: Blue line detection (speed up on fast track)
    if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
      currentSpeed = FAST_SPEED;
    } else {
      currentSpeed = BASE_SPEED;
    }
    
    // Wall follow with current speed
    int POut = (int)PDcontroller.update(wallDist, distFromWall);
    motors.setSpeeds(int(currentSpeed + POut), int(currentSpeed - POut));
  }
  
  motors.setSpeeds(0, 0);
  currentSpeed = BASE_SPEED;  // Reset to normal speed
}

void wallFollowRight(float targetDistance) {
  Serial.print("Wall follow RIGHT for ");
  Serial.print(targetDistance);
  Serial.println("cm");
  
  servo.write(0);
  
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;
  
  while(distanceTraveled < targetDistance) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;
    
    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;
    
    wallDist = sonar.readDist();
    
    // ===== CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];
    
    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD && pickCount < TARGET_PICKS) {
      Serial.println("BLACK SQUARE DETECTED! Picking...");
      motors.setSpeeds(0, 0);
      delay(100);
      
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);
      
      pickCount++;
      display.clear();
      display.print("Picks: ");
      display.print(pickCount);
      flash_led();
      
      delay(500);
      
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      leftTotal = 0;
      rightTotal = 0;
      distanceTraveled = 0;
      continue;
    }
    
    // PHASE 4: Blue line detection (speed up on fast track)
    if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
      currentSpeed = FAST_SPEED;
    } else {
      currentSpeed = BASE_SPEED;
    }
    
    // Wall follow with current speed
    int POut = (int)PDcontroller.update(wallDist, distFromWall);
    motors.setSpeeds(int(currentSpeed - POut), int(currentSpeed + POut));
  }
  
  motors.setSpeeds(0, 0);
  currentSpeed = BASE_SPEED;  // Reset to normal speed
}

// ====================== TURN FUNCTIONS ======================
void leftTurn() {
  Serial.println("TURN_LEFT");
  motors.setSpeeds(50, -50);
  delay(1800);
  motors.setSpeeds(0, 0);
  delay(200);
}

void rightTurn() {
  Serial.println("TURN_RIGHT");
  motors.setSpeeds(-50, 50);
  delay(1800);
  motors.setSpeeds(0, 0);
  delay(200);
}

void uTurn() {
  Serial.println("Executing U-TURN...");
  motors.setSpeeds(-50, 50);
  delay(3600);
  motors.setSpeeds(0, 0);
  delay(100);
}

// ====================== SENSOR CALIBRATION ======================
void calibrateSensors() {
  delay(1000);
  Serial.println("Calibrating sensors...");
  
  for(int i = 0; i < 120; i++) {
    if (i > 30 && i <= 90) {
      motors.setSpeeds(-60, 60);  // Rotate left
    } else {
      motors.setSpeeds(60, -60);  // Rotate right
    }
    lineSensors.calibrate();
  }
  
  motors.setSpeeds(0, 0);
  Serial.println("Calibration complete!");
}

// ====================== HELPER FUNCTIONS ======================
void flash_led() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  delay(200);
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
}
