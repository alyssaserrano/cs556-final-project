#include <Pololu3piPlus32U4.h>
#include "odometry.h"
#include "sonar.h"
#include <Servo.h>
#include "PDcontroller.h"
#include "PIDcontroller.h"
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
#define kp 1.0
#define kd 0.6
#define BASE_SPEED 100
#define FAST_SPEED 200
#define WALL_DIST 10
#define FRONT_OBSTACLE_DIST 10

// ======================== PID VARIABLES ==================
#define minOutputVel -200
#define maxOutputVel 200
#define kpVel 30 //Tune Kp here
#define kdVel 10 //Tune Kd here
#define kiVel 7 //Tune Ki here
#define clamp_iVel 10 //Tune ki integral clamp here
#define base_speedVel 200

// ============= Phase 3/4: IR Sensor Detection =================
#define BLUE_MIN_CAL 200
#define BLUE_MAX_CAL 500
#define BLACK_THRESHOLD 900

// ====================== HARDWARE OBJECTS ======================
Motors motors;
Encoders encoders;
Sonar sonar(4);
PDcontroller PDcontroller(kp, kd, minOutput, maxOutput);
PIDcontroller PIDcontroller(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
LineSensors lineSensors;
OLED display;
Servo servo;

// ====================== STATE VARIABLES ======================
float frontWallDist;
float currentDistFromWall;
float desiredDistFromWall = WALL_DIST;
int PDout = 0;
int currentSpeed = BASE_SPEED;
long deltaL = 0;
long deltaR = 0;
const float goal_theta;
float theta;
int pickCount = 0;
int lineCenter = 2000;

// ==================== MAP VARIABLES ============================
float gridSize = 20.0;

// FSA (Finite States)
enum State { EXIT_DOCK, EXPLORE, PICK_SERVICE, CHECK_GOALS, RETURN_HOME, DOCK_ALIGN, COMPLETE, LINE_FOLLOWING, PREPARE_HOME };
State currentMode;

// Sensor readings
unsigned int lineDetectionValues[5];

// ====================== SETUP (INIT) ======================
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

  currentMode = EXIT_DOCK;
}

// ====================== MAIN CONTROL LOOP ======================
void loop() {
  Serial.println("===== FSA BEGIN!!! =====");

  uint16_t cal[5];  // Declare ONCE for the switch cases
  uint16_t linePos;
  
    // FSA Logic
    switch(currentMode){
      case EXIT_DOCK:
        moveForward(20.0);
        currentMode = EXPLORE;
        break;

      case EXPLORE:
        // Mark down visited cell
        // Localizes with odometry + controller.
        explore();

        // Check to make sure if there is no wall in front.
        checkFrontWall();

        // Check center of square for black if it is, IMMEDIATELY execute next state.
        if(blackSquareDetected()){
          currentMode = PICK_SERVICE;
          break;
        }
        
        // Check if there is a blue line.
        lineSensors.readCalibrated(cal);

        if(computeBlueLinePosition(cal, linePos)){
          currentMode = LINE_FOLLOWING;
          break;
        }
        break;

      case LINE_FOLLOWING:
        lineFollowing();
        lineSensors.readCalibrated(cal);
        
        if(! computeBlueLinePosition(cal, linePos)){  // Checks if OFF the line
          currentMode = EXPLORE;
        }
        break;

      case PICK_SERVICE:
        Serial.println("State: Pick_service!");
        // TODO: pick_service();
        pickCount++;
        currentMode = CHECK_GOALS;
        break;

      case CHECK_GOALS:
        // Check for all goals
        if(pickCount < 3){
          currentMode = EXPLORE;
        }
        else{
          currentMode = PREPARE_HOME;
        }
        break;

      case PREPARE_HOME:
        // Turn 180 to prepare for right wall follow
        // Turn left 2x which equals 180 degress
        leftTurn();
        leftTurn();
        currentMode = RETURN_HOME;
        break;

      case RETURN_HOME:
        // Move per grid, check if there is a front wall.
        returnHome();

        // Check if there is a blue line.
        lineSensors.readCalibrated(cal);

        // Zoom if we have detected the blue line.
        if(computeBlueLinePosition(cal, linePos)){
          currentMode = LINE_FOLLOWING;
        }
        break;
      // TODO: DOCK ALIGN, identify the half black half white square.
      case DOCK_ALIGN:
        //dockAlign();
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
  delay(10);

}

// ====================== WALL FOLLOW  ======================
void wallLeft(float targetDist) {
  // Look left
  servo.write(180);

  // Ensure proper reset.
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  // Continue to travel till you reach the goal distance.
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
    //Serial.println(currentDistFromWall);


    // Get PD output or PID output
    // Remember placedholders: PDcontroller.update(ACTUAL DISTANCE, IDEAL TARGET DISTANCE TO MAINTAIN)
    PDout = PDcontroller.update(currentDistFromWall, desiredDistFromWall);
    // Uncomment when wanting PID option.
    //PDout = PIDcontroller.update(currentDistFromWall, desiredDistFromWall);

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
  delay(500);
}

void wallRight(float targetDist) {
  // Look right
  servo.write(0);

  // Ensure proper reset.
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  // Continue to travel till you reach the goal distance.
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
    //Serial.println(currentDistFromWall);


    // Get PD or PID output to adjust wheel speed.
    // Remember placedholders: PDcontroller.update(ACTUAL DISTANCE, IDEAL TARGET DISTANCE TO MAINTAIN)
    PDout = PDcontroller.update(currentDistFromWall, desiredDistFromWall);
    // Uncomment when wanting PID option.
    //PDout = PIDcontroller.update(currentDistFromWall, desiredDistFromWall);

    // Left = negative, right = positive
    int leftCmd = (int)(currentSpeed - PDout);
    int rightCmd = (int)(currentSpeed + PDout);

    // Constrain max 400 speed
    leftCmd  = constrain(leftCmd,  -400, 400);
    rightCmd = constrain(rightCmd, -400, 400);

    motors.setSpeeds(leftCmd, rightCmd);
  }
  
  motors.setSpeeds(0, 0);
  delay(10);
}


// ====================== TURN FUNCTIONS ======================
void rightTurn() {
  Serial.println("RIGHT TURN");
  motors.setSpeeds(50, -50);
  delay(1800);
  motors.setSpeeds(0, 0);
  delay(200);
}

void leftTurn() {
  Serial.println("LEFT TURN");
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
  Serial.println("WallLeft Called");
  wallLeft(gridSize);

  // TODO: Mark correct square as visited
  
}

// ================ FRONT WALL OBSTACLE AVOIDANCE ==============================
bool checkFrontWall(){

  // Incrementally check for front wall
  servo.write(90);
  delay(500);
  frontWallDist = sonar.readDist();

  // This shows there is a front wall, therefore we have to turn the robot body if we are still looking for goals.
  // Else the robot's body will be oriented for right follow therefore having to make left turns rather than right.
  if(frontWallDist < FRONT_OBSTACLE_DIST && currentMode != RETURN_HOME){
    rightTurn();
  }
  else{
    leftTurn();
  }
}

bool blackSquareDetected(){
  uint16_t cal[5];
  lineSensors.readCalibrated(cal);
  
  // Check center sensor for black square
  uint16_t centerVal = cal[2];
  
  if(centerVal >= BLACK_THRESHOLD){
    Serial.println("BLACK SQUARE DETECTED!");
    return true;
  }
  return false;

}

//====================== LINE FOLLOW ==========================
bool computeBlueLinePosition(uint16_t cal[5], uint16_t &position){  
  uint32_t weightedSum = 0;
  uint32_t total = 0;

  for(int i = 0; i < 5; i++){
    int val = cal[i];
    
    if(val >= BLUE_MIN_CAL && val <= BLUE_MAX_CAL){
      int v = val - BLUE_MIN_CAL;
      int pos = i * 1000;
      
      weightedSum += (uint32_t)v * pos;
      total += v;
    }
  }

  if(total == 0){
    return false;
  }

  position = weightedSum / total;
  return true;
}

void lineFollowing(){
  uint16_t cal[5];
  lineSensors.readCalibrated(cal);

  uint16_t position;
  computeBlueLinePosition(cal, position);


  // Target center position
  int PDout = PDcontroller.update(position, lineCenter);

  int leftSpeed = FAST_SPEED + PDout;
  int rightSpeed = FAST_SPEED - PDout;
  
  // Constrain speeds
  leftSpeed = constrain(leftSpeed, -400, 400);
  rightSpeed = constrain(rightSpeed, -400, 400);
  
  motors.setSpeeds(leftSpeed, rightSpeed);

  Serial.print("FOLLOWING - Pos: ");
  Serial.print(position);
  Serial.print(" PD: ");
  Serial.println(PDout);
}

// ====================== Pick service duties =====================
/*void pick_service(){
  // TODO: REVIEW CODE

}*/

// ================= Return home ================================
void returnHome(){

  // To go home we need to do the same approach as explore/navigate but with right wall follow.
  // Again localizing with odometry.
  wallRight(gridSize);

  checkFrontWall();
}
