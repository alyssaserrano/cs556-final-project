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
#define kp 2.0
#define kd 0.4
#define BASE_SPEED 100
#define FAST_SPEED 200
#define WALL_DIST 20
#define FRONT_OBSTACLE_DIST 15

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
float currentDistFromWall;
float desiredDistFromWall = WALL_DIST;
int PDout = 0;
int currentSpeed = BASE_SPEED;
long deltaL = 0;
long deltaR = 0;
const float goal_theta;
float theta;
int pickCount = 0;

// FSA (Finite States)
enum State { EXIT_DOCK, EXPLORE, PICK_SERVICE, CHECK_GOALS, RETURN_HOME, DOCK_ALIGN, COMPLETE, LINE_FOLLOWING };
State currentMode;

// Sensor readings
unsigned int lineDetectionValues[5];

// ====================== SETUP (INIT) ======================
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

  currentMode = EXIT_DOCK;
}

// ====================== MAIN CONTROL LOOP ======================
void loop() {
  Serial.println("=== FORWARD PATH ===");
  
    switch(currentMode){
      case EXIT_DOCK:
        moveForward(20.0);
        currentMode = EXPLORE;
        break;
      case EXPLORE:
        // Mark down visited cell
        explore();

        // Check center of square for black.
        if(blackSquareDetected()){
          currentMode = PICK_SERVICE;
        }

        // Check if there is a blue line.
        if(computeBlueLinePosition()){
          currentMode = LINE_FOLLOWING;
        }
        break;
      case LINE_FOLLOWING:
        linefollowing();
        leftTurn();
        currentMode = EXPLORE;

        break;
      case PICK_SERVICE:
        Serial.println("State: Pick_service!");
        pick_service();
        pickCount++;
      
        currentMode = CHECK_GOALS;
        break;

      case CHECK_GOALS:
        // Check for all goals
        if(pickCount < 3){
          currentMode = EXPLORE;
        }
        else{
          currentMode = RETURN_HOME;
        }
        break;

      case RETURN_HOME:
        returnHome();

        if(nearDock()){
          currentMode = DOCK_ALIGN;
        }
        break;

      case DOCK_ALIGN:
        dockAlign();

        currentMode = COMPLETE;
        break;
      case COMPLETE:
        Serial.println("STATE: COMPLETE");
        motors.setSpeeds(0, 0);
        break;
  }

  Serial.println("=== MISSION COMPLETE!  ===");
}


// =============== MOVE FORWARD ================================
void moveForward(float targetDist) {
  // Reset encoders to get accurate target distance
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

    // Update theta
    theta = (deltaR - deltaL) / w;

    // Adjust theta to correct itself to be in a straight line.
    PDout = PDcontroller.update(theta, goal_theta);
    motors.setSpeeds(int(currentSpeed), int(currentSpeed));
  }

  // Stop
  motors.setSpeeds(0, 0);

}

// ====================== WALL FOLLOW LEFT ======================
void wallLeft(float targetDist) {
  // Look left
  servo.write(180);

  // Ensure proper reset.
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

    currentDistFromWall = sonar.readDist();
    Serial.println(currentDistFromWall);


    // Get PD output
    PDout = PDcontroller.update(currentDistFromWall, desiredDistFromWall);

    // Left = positive, right = negative
    int leftCmd = (int)(currentSpeed + PDout);
    int rightCmd = (int)(currentSpeed - PDout);

    // Constrain max 400 speed
    leftCmd  = constrain(leftCmd,  -400, 400);
    rightCmd = constrain(rightCmd, -400, 400);

    motors.setSpeeds(leftCmd, rightCmd);
  }

  // After task is done stop motors.
  motors.setSpeeds(0, 0);
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

// ======================= EXPLORE ============================
void explore(){
  // Wall follow per grid
  wallLeft(20.0);

  // Mark correct square as visited
  
}

void blackSquareDetected(){

}

//====================== LINE FOLLOW ==========================
bool computeBlueLinePosition(){

}

void linefollowing(){

}

// ====================== Pick service duties =====================
void pick_service(){

}

// ================= Return home ================================
void returnHome(){

}