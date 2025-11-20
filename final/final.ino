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


Map worldMap; // one map for the program

ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

#define PI 3.14159

#define lenOfMap 60 //60 cm / 3 = 20 cm per grid unit (matches the map).
#define N_particles 25
#define move_noise   0.01   // σ_d^2
#define rotate_noise 0.05   // σ_θ^2
#define ultra_noise  0.10   // σ_s^2

#define diaL 3.2 //define physical robot parameters
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

Motors motors;
Encoders encoders;

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
ParticleFilter particle(worldMap, lenOfMap, N_particles, move_noise, rotate_noise, ultra_noise);
Sonar sonar(4);

uint8_t iter = 0;
uint8_t movement_cycles = 0;
bool is_localized = false;

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10;
float y = 10;
float theta = 0.0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;


//PID Movement
#define kp 200
#define kd 5
#define ki 0.5
#define clamp_i 50
#define base_speed 100
#define minOutput -100
#define maxOutput 100

static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

PIDcontroller PIDcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
double PID_OUT_ANGLE, PID_OUT_DISTANCE;

// Wall detection threshold (in cm)
#define WALL_THRESHOLD 15.0

void setup() {
    Serial.begin(9600);
    while (!Serial) continue;
    delay(1000);

  //seed RNG for diverse particles.
  randomSeed(analogRead(A0));  
 
  float origin[2] = {0.5, 0.5};
  theta = 0.0f;
  float closestDist = worldMap.closest_distance(origin, theta);
  Serial.println("Map test distance:");
  Serial.println(closestDist);
  
  Serial.println("Starting autonomous localization...");
  delay(2000);
}

void loop() {
  // Check if robot is confident about its location
  if (!is_localized) {
    // Perform sensing and movement
    sensing_and_movement();
    movement_cycles++;
    
    //Get odometer readings  
    CalculateDistance();

    //Propagate particles by using move_particles.
    float dx = x - x_last;
    float dy = y - y_last;
    float dtheta = wrapPi(theta - theta_last);
    particle.move_particles(dx, dy, dtheta);

    //Measure, estimation, and resample
    particle.measure();

    // Display all particle locations and estimated robot location on screen   
    particle.print_particles();
    
    //save last odometer reading
    x_last = x;
    y_last = y;
    theta_last = theta;
    
    // Check confidence after minimum 20 cycles
    if (movement_cycles >= 20) {
      if (confidence()) {
        is_localized = true;
        motors.setSpeeds(0, 0);
        Serial.println("========================================");
        Serial.println("LOCALIZATION COMPLETE!");
        Serial.println("Robot is confident about its position.");
        Serial.println("========================================");
      }
    }
    
    iter++;
    delay(1000); //for easier viewing of output
  } else {
    // Robot is localized, stop
    motors.setSpeeds(0, 0);
    delay(5000); // Wait before potentially restarting
  }
}

// STEP 1: Robot Movement Behavior Logic
void sensing_and_movement() {
  // Read sonar distance
  float front_dist = sonar.readDist();
  
  Serial.print("Front distance: ");
  Serial.print(front_dist);
  Serial.println(" cm");
  
  // Strategy: Wall-following with right-hand rule
  // Priority: 1) Avoid front wall, 2) Keep wall on right, 3) Move forward
  
  if (front_dist < WALL_THRESHOLD) {
    // Wall in front - turn left
    Serial.println("Wall ahead - turning left");
    turn_left();
  } else {
    // No wall in front - move forward
    Serial.println("Path clear - moving forward");
    move_forward();
  }
}

// STEP 2: Determining Localization Confidence
bool confidence() {
  // Get particle positions and count particles in same grid cell
  const float grid_size = lenOfMap / 3.0; // 20 cm per grid
  
  // Create a simple grid counter (3x3 grid = 9 cells)
  uint8_t grid_count[3][3] = {0};
  
  // Count particles in each grid cell
  for (uint8_t i = 0; i < N_particles; i++) {
    float px = particle.get_particle_x(i);
    float py = particle.get_particle_y(i);
    
    // Convert to grid coordinates
    int grid_x = (int)(px / grid_size);
    int grid_y = (int)(py / grid_size);
    
    // Clamp to valid grid range [0, 2]
    if (grid_x < 0) grid_x = 0;
    if (grid_x > 2) grid_x = 2;
    if (grid_y < 0) grid_y = 0;
    if (grid_y > 2) grid_y = 2;
    
    grid_count[grid_x][grid_y]++;
  }
  
  // Find maximum concentration in any grid cell or neighboring cells
  uint8_t max_cluster = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      uint8_t cluster_count = grid_count[i][j];
      
      // Add neighboring cells to cluster count
      if (i > 0) cluster_count += grid_count[i-1][j];
      if (i < 2) cluster_count += grid_count[i+1][j];
      if (j > 0) cluster_count += grid_count[i][j-1];
      if (j < 2) cluster_count += grid_count[i][j+1];
      
      if (cluster_count > max_cluster) {
        max_cluster = cluster_count;
      }
    }
  }
  
  Serial.print("Max cluster size: ");
  Serial.print(max_cluster);
  Serial.print(" / ");
  Serial.println(N_particles);
  
  // Confidence threshold: >88% of particles (22 out of 25)
  return (max_cluster >= 22);
}

// Movement primitives
void move_forward() {
  motors.setSpeeds(100, 100);
  delay(1500); // Move for ~1.5 seconds
  motors.setSpeeds(0, 0);
  delay(200);
}

void turn_left() {
  const float turnLeft_rad = PI / 2.0f;
  const float tol_rad = 0.05f;
  float theta_start = theta;
  float theta_target = wrapPi(theta_start + turnLeft_rad);
  
  unsigned long start_time = millis();
  unsigned long timeout = 3000; // 3 second timeout
  
  while (millis() - start_time < timeout) {
    CalculateDistance();
    float err = wrapPi(theta_target - theta);
    
    PID_OUT_ANGLE = PIDcontroller.update(theta, theta_target);
    motors.setSpeeds((int)PID_OUT_ANGLE, (int)-PID_OUT_ANGLE);
    
    if (fabsf(err) <= tol_rad) {
      motors.setSpeeds(0, 0);
      break;
    }
    delay(20);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

void turn_right() {
  const float turnRight_rad = -PI / 2.0f;
  const float tol_rad = 0.05f;
  float theta_start = theta;
  float theta_target = wrapPi(theta_start + turnRight_rad);
  
  unsigned long start_time = millis();
  unsigned long timeout = 3000;
  
  while (millis() - start_time < timeout) {
    CalculateDistance();
    float err = wrapPi(theta_target - theta);
    
    PID_OUT_ANGLE = PIDcontroller.update(theta, theta_target);
    motors.setSpeeds((int)-PID_OUT_ANGLE, (int)PID_OUT_ANGLE);
    
    if (fabsf(err) <= tol_rad) {
      motors.setSpeeds(0, 0);
      break;
    }
    delay(20);
  }
  motors.setSpeeds(0, 0);
  delay(200);
}

void CalculateDistance(){
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;  
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}
