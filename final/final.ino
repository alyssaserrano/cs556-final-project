#include <Pololu3piPlus32U4.h>
#include "odometry. h"
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
#define FAST_SPEED 250
#define WALL_DIST 20

// Phase 3/4: IR Sensor Detection
#define BLUE_MIN_CAL 200
#define BLUE_MAX_CAL 500
#define BLACK_THRESHOLD 900
#define TARGET_PICKS 3

// Segment types
#define WALL_FOLLOW_LEFT 0
#define WALL_FOLLOW_RIGHT 1
#define LEFT_TURN 2
#define RIGHT_TURN 3
#define U_TURN 4

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
int pickCount = 0;
long deltaL = 0;
long deltaR = 0;

// Sensor readings
unsigned int lineDetectionValues[5];

// ====================== SEGMENT STRUCT ======================
struct Segment {
  int type;      // WALL_FOLLOW_LEFT, WALL_FOLLOW_RIGHT, LEFT_TURN, RIGHT_TURN, U_TURN
  float distance; // distance in cm (0 for turns)
};

// ====================== FORWARD PATH ======================
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

const int NUM_FORWARD_SEGMENTS = 33;

// ====================== REVERSE PATH (REVERSED & SWAPPED) ======================
Segment reversePath[] = {
  {WALL_FOLLOW_RIGHT, 20},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 20},
  {U_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 60},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 40},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 60},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {U_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 60},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 40},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 20},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 40},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_RIGHT, 20},
  {RIGHT_TURN, 0},
  {WALL_FOLLOW_LEFT, 100},
  {LEFT_TURN, 0},
  {WALL_FOLLOW_LEFT, 60}
};

const int NUM_REVERSE_SEGMENTS = 33;

// ====================== SETUP ======================
void setup() {
  Serial.begin(9600);
  delay(2000);
  servo.attach(5);
  servo.write(180);
  delay(40);

  // Calibrate line sensors for Phase 3/4
  Serial.println("Calibrating sensors...");
  calibrateSensors();
  delay(1000);
}

// ====================== MAIN CONTROL LOOP ======================
void loop() {
  Serial.println("=== STARTING FORWARD PATH ===");
  
  // Execute forward path with Phase 3/4 detection
  for (int i = 0; i < NUM_FORWARD_SEGMENTS && pickCount < TARGET_PICKS; i++) {
    execute_segment(forwardPath[i]);
    delay(500);
  }

  // Once 3 picks found, reverse
  if (pickCount == TARGET_PICKS) {
    Serial.println("=== 3 PICKS FOUND!  RETURNING HOME ===");
    delay(2000);

    // Execute reverse path
    for (int i = 0; i < NUM_REVERSE_SEGMENTS; i++) {
      execute_segment(reversePath[i]);
      delay(500);
    }

    Serial.println("=== MISSION COMPLETE!  ===");
    while(1);  // Stop
  }
}

// ====================== EXECUTE SEGMENT ======================
void execute_segment(Segment seg) {
  if (seg.type == WALL_FOLLOW_LEFT) {
    wallFollowLeft(seg.distance);
  } 
  else if (seg.type == WALL_FOLLOW_RIGHT) {
    wallFollowRight(seg.distance);
  } 
  else if (seg. type == LEFT_TURN) {
    leftTurn();
  } 
  else if (seg.type == RIGHT_TURN) {
    rightTurn();
  } 
  else if (seg.type == U_TURN) {
    uTurn();
  }
}

// ====================== WALL FOLLOWING WITH PHASE 3/4 ======================
void wallFollowLeft(float targetDistance) {
  Serial.print("Wall LEFT for ");
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
    deltaR = encoders. getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;

    float countsPerRev = nL * gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;

    wallDist = sonar.readDist();

    // ===== PHASE 3/4: CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];

    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD && pickCount < TARGET_PICKS) {
      Serial.println("BLACK SQUARE DETECTED! Picking.. .");
      motors.setSpeeds(0, 0);
      delay(200);

      // 360° spin
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);

      pickCount++;
      Serial.print("Picks: ");
      Serial.println(pickCount);

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

    // Wall follow with current speed
    PDout = PDcontroller.update(wallDist, distFromWall);
    motors.setSpeeds(int(currentSpeed - PDout), int(currentSpeed + PDout));
  }

  motors.setSpeeds(0, 0);
  currentSpeed = BASE_SPEED;
}

void wallFollowRight(float targetDistance) {
  Serial.print("Wall RIGHT for ");
  Serial. print(targetDistance);
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

    // ===== PHASE 3/4: CHECK SURFACE TYPE =====
    lineSensors.readCalibrated(lineDetectionValues);
    uint16_t centerSensor = lineDetectionValues[2];

    // PHASE 3: Black square detection
    if (centerSensor >= BLACK_THRESHOLD && pickCount < TARGET_PICKS) {
      Serial.println("BLACK SQUARE DETECTED! Picking...");
      motors.setSpeeds(0, 0);
      delay(200);

      // 360° spin
      motors.setSpeeds(50, -50);
      delay(3600);
      motors.setSpeeds(0, 0);
      delay(200);

      pickCount++;
      Serial.print("Picks: ");
      Serial.println(pickCount);

      // Reset distance tracking
      encoders.getCountsAndResetLeft();
      encoders. getCountsAndResetRight();
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

    // Wall follow with current speed
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
  motors.setSpeeds(-50, 50);
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
  Serial.println("Calibrating sensors.. .");
  
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
