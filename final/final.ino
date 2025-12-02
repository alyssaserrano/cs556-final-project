#include <Pololu3piPlus32U4.h>
#include "odometry.h"
#include "sonar.h"
#include <Servo.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

// ----------------- Set up ----------------------------------------//
#define PI 3.14159

#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

// kp and kd
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
  // Go forward 60 cm.
  wallRight(60.0);
  delay(500);

  // Then turn left
  leftTurn();
  delay(500);

  // Go forward another 80 to first goal pos
  // 160 cm - 20 cm = 80 cm
  wallRight(80.0);
  delay(500);

  // Turn left at (100, 80)
  leftTurn();
  delay(500);

  // go forward 40
  wallRight(40.0);
  delay(500);

  // turn right at (100, 20)
  rightTurn();
  delay(500);

  // go forward 40 cm
  wallRight(40.0);
  delay(500);

  // turn right at (120, 20)
  rightTurn();
  delay(500);

  // go forward 40 cm
  wallRight(40.0);
  delay(500);

  // Turn left at (120, 60)
  leftTurn();
  delay(500);

  // go forward 20 cm
  wallRight(20.0);
  delay(500);

  // turn left
  leftTurn();
  delay(500);

  // go forward 60 cm
  wallRight(60.0);
  delay(500);

  // turn left at (180, 0)
  leftTurn();
  delay(500);

  // go forward 10 cm (toward alley way)
  wallRight(10.0);
  delay(500);

  // turn left
}

// -------------------------------------------- Core Behaviors --------------------------------------//
void wallLeft(float targetDist){
  // face head to left
  servo.write(180);

  // make sure to reset encoders count and other variables for accurate measurement
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  float distanceTraveled = 0;
  int16_t leftTotal = 0;
  int16_t rightTotal = 0;

  // Loop to get calculate the distance traveled till we reach out desired distance
  while(distanceTraveled < targetDistance){
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    leftTotal += deltaL;
    rightTotal += deltaR;

    float countsPerRev = nL + gearRatio;
    float distPerCount = (PI * diaL) / countsPerRev;
    float avgCounts = (leftTotal + rightTotal) / 2.0;
    distanceTraveled = avgCounts * distPerCount;

    wallDist = sonar.readDist();
    Serialprintln(wallDist);

    // Get PD output to set the speed of the motors
    PDout = PDcontroller.update(wallDist, distFromWall);
    // Made for left wall following
    motors.setSpeeds(int (baseSpeed - PDout), int(baseSpeed + PDout));

  }

}

