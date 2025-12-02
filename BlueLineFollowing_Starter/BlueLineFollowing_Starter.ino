// =============================================================================
// Blue Line Following Starter Template
// Pololu 3pi+ 32U4 Robot
// =============================================================================
// 
// OVERVIEW:
// This program uses the IR line sensors to detect and follow a blue line,
// while also identifying black squares as "service locations."
//
// The calibrated IR sensors return values roughly in ranges:
//   - Floor (white/light):  Low values (near 0)
//   - Blue tape/line:       Medium values (~200-600 after calibration)
//   - Black square:         High values (near 1000)
//
// YOUR TASKS:
//   1. In setup(): Initialize sensors by calling calibration
//   2. In loop(): Read sensors, identify surface type, and respond accordingly
//
// =============================================================================

#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"

using namespace Pololu3piPlus32U4;

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
LineSensors lineSensors;
Motors motors;

// =============================================================================
// PD CONTROLLER CONFIGURATION
// =============================================================================
// These values control how aggressively the robot corrects its path
#define minOutput -100      // Minimum correction value
#define maxOutput 100       // Maximum correction value
#define baseSpeed 100       // Base motor speed when following line
#define kp_line 0.25        // Proportional gain
#define kd_line 1           // Derivative gain

// Create PD controller instance for line following
PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);

// =============================================================================
// CALIBRATION SETTINGS
// =============================================================================
int calibrationSpeed = 60;  // Speed during calibration rotation

// =============================================================================
// LINE FOLLOWING SETTINGS
// =============================================================================
int lineCenter = 2000;      // Target position (center of 0-4000 range)

// =============================================================================
// COLOR DETECTION THRESHOLDS
// =============================================================================
// These thresholds are used to classify what surface the robot is over
// based on calibrated sensor readings.
//
// Tune these values based on your specific environment and tape colors!
//
// BLUE line detection range (calibrated values typically 200-600)
const uint16_t BLUE_MIN_CAL = 200;
const uint16_t BLUE_MAX_CAL = 500;

// BLACK square threshold (calibrated values typically > 900)
const uint16_t BLACK_THRESHOLD = 900;


// =============================================================================
// FUNCTION: calibrateSensors()
// =============================================================================
// Purpose: Calibrate the IR line sensors by rotating the robot left and right
//          over the different surfaces (floor, blue line, black square).
//
// How it works:
//   - Robot rotates right for 30 iterations
//   - Robot rotates left for 60 iterations  
//   - Robot rotates right for 30 iterations
//   - During rotation, sensors record min/max values for calibration
//
// Call this once in setup() before attempting line following!
// =============================================================================
void calibrateSensors()
{
  delay(1000);
  Serial.println("Calibrating sensors...");   
  
  for(int i = 0; i < 120; i++){
    if (i > 30 && i <= 90){
      // Rotate left
      motors.setSpeeds(int(-1 * calibrationSpeed), calibrationSpeed);
    }
    else{
      // Rotate right
      motors.setSpeeds(calibrationSpeed, int(-1 * calibrationSpeed));
    }
    lineSensors.calibrate();
  } 
  
  motors.setSpeeds(0, 0);
  Serial.println("Calibration complete!");
}


// =============================================================================
// FUNCTION: computeBlueLinePosition()
// =============================================================================
// Purpose: Analyze calibrated sensor readings to determine WHERE the blue line
//          is positioned under the robot (if present).
//
// Parameters:
//   cal[5]   - Array of 5 calibrated sensor values (input)
//   position - Will be set to line position 0-4000 if found (output)
//
// Returns:
//   true  - Blue line was detected, position is valid
//   false - No blue line detected
//
// Position Scale:
//   0    = Line is under far LEFT sensor
//   1000 = Line is under sensor 1
//   2000 = Line is under CENTER sensor (sensor 2)
//   3000 = Line is under sensor 3
//   4000 = Line is under far RIGHT sensor
//
// Algorithm:
//   Uses weighted average of sensors reading "blue" values to estimate
//   the line's position. Sensors with stronger blue readings contribute
//   more to the position calculation.
// =============================================================================
bool computeBlueLinePosition(uint16_t cal[5], uint16_t &position){
  // Accumulators for weighted-average calculation
  uint32_t weightedSum = 0;   // Sum of (activation * sensor_position)
  uint32_t total = 0;         // Sum of activations

  // Check each of the 5 sensors
  for(int i = 0; i < 5; i++){
    uint16_t val = cal[i];    // Calibrated value for sensor i 

    // Only consider sensors reading in the "blue" range
    if(val >= BLUE_MIN_CAL && val <= BLUE_MAX_CAL){
      // Calculate activation strength (how "blue" the reading is)
      // Higher values within the blue range = stronger activation
      uint16_t v = val - BLUE_MIN_CAL;
      
      // Calculate this sensor's position on 0-4000 scale
      // Sensor 0 = position 0, Sensor 1 = 1000, etc.
      uint16_t pos = i * 1000;

      // Add this sensor's contribution to weighted sum
      weightedSum += (uint32_t)v * pos;
      total += v;
    }
  }

  // If no sensor detected blue, return false
  if(total == 0){
    return false;
  }

  // Calculate weighted average position
  position = weightedSum / total;
  return true;
}


// =============================================================================
// FUNCTION: lineFollowing()
// =============================================================================
// Purpose: Execute one iteration of PD-controlled line following.
//
// How it works:
//   1. Read calibrated sensor values
//   2. Compute blue line position using computeBlueLinePosition()
//   3. If line found: Use PD controller to calculate motor correction
//   4. If line lost: Stop motors
//
// The PD controller compares current position to lineCenter (2000) and
// outputs a correction value that adjusts left/right motor speeds.
// =============================================================================
void lineFollowing(){
  // Read current calibrated sensor values
  uint16_t cal[5];
  lineSensors.readCalibrated(cal);
  
  // Try to find the blue line position
  uint16_t position;
  bool found = computeBlueLinePosition(cal, position);

  // If blue line not detected, stop and return
  if(!found){
    Serial.println("BLUE LINE LOST - Stopping");
    motors.setSpeeds(0, 0);
    return;
  }

  // Calculate PD correction based on position error
  // If position < 2000: line is left of center, need to turn left
  // If position > 2000: line is right of center, need to turn right
  int PDout = pd_line.update(position, lineCenter);

  // Apply correction to motor speeds
  // PDout > 0 means line is right, so speed up left motor
  // PDout < 0 means line is left, so speed up right motor
  int leftSpeed = baseSpeed + PDout;
  int rightSpeed = baseSpeed - PDout;
  
  motors.setSpeeds(leftSpeed, rightSpeed);

  // Debug output
  Serial.print("FOLLOWING - Pos: ");
  Serial.print(position);
  Serial.print(" PD: ");
  Serial.print(PDout);
  Serial.print(" L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
}


// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(9600);
  delay(2000);  // Wait for serial monitor and robot to stabilize

  // =========================================================================
  // TODO 1: Initialize the IR sensors
  // =========================================================================
  // Before the robot can follow a line, the sensors must be calibrated.
  // This teaches the sensors what "floor", "blue", and "black" look like.
  //
  // HINT: There is a function defined above that handles calibration.
  //       You need to call it here.
  // =========================================================================
  
  // YOUR CODE HERE:
  calibrateSensors();
}


// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  

  // =========================================================================
  // TODO 2: Declare an array to store sensor readings
  // =========================================================================
  // The Pololu 3pi+ has 5 IR line sensors. You need an array to hold
  // the calibrated reading from each sensor.
  //
  // HINT: The array should hold 5 values of type uint16_t
  // =========================================================================
  
  // YOUR CODE HERE:
  uint16_t cal[5];


  // =========================================================================
  // TODO 3: Read the calibrated sensor values
  // =========================================================================
  // Use the lineSensors object to read calibrated values into your array.
  //
  // HINT: The function is lineSensors.readCalibrated() and you pass it
  //       your array as the argument.
  // =========================================================================
  
  // YOUR CODE HERE:
  lineSensors.readCalibrated(cal);
  

  // =========================================================================
  // TODO 4: Get the center sensor's calibrated value
  // =========================================================================
  // For detecting black squares vs blue line vs floor, we primarily
  // look at the CENTER sensor (index 2 in a 0-indexed array).
  //
  // HINT: Create a uint16_t variable and assign it the value from
  //       the center position of your sensor array.
  // =========================================================================
  
  // YOUR CODE HERE:
  uint16_t centerSensor = cal[2];
  

  // =========================================================================
  // TODO 5: Add debug output (optional but helpful!)
  // =========================================================================
  // Print the center calibrated value to Serial so you can see what
  // values correspond to floor, blue line, and black squares.
  //
  // HINT: Use Serial.print() to output the center sensor value
  // =========================================================================
  
  // YOUR CODE HERE (optional):
  

  // =========================================================================
  // TODO 6: Implement surface detection and response logic
  // =========================================================================
  // Based on the center sensor's calibrated value, determine what surface
  // the robot is over and respond appropriately:
  //
  //   A) BLACK SQUARE (Service Location):
  //      - Condition: centerCal >= BLACK_THRESHOLD
  //      - Action: transition to a new state to perform tasks related to service (Phase 3)
  //      - This represents a destination/service point
  //
  //   B) BLUE LINE:
  //      - Condition: centerCal is between BLUE_MIN_CAL and BLUE_MAX_CAL
  //      - Action: Call the lineFollowing() function to speed up and perform tasks related to fast track (Phase 4)
  //      - Robot should follow the blue line
  //
  //   C) FLOOR (off the line):
  //      - Condition: Neither of the above (else case)
  //      - Action: reset the speed to the basespeed; transition to one of the stated related to Navigation and Localization (phase 1) or Return to Staging/Charging Dock (Phase )
  //
  // HINT: Use if / else if / else structure with the threshold constants
  //       defined at the top of this file.
  // =========================================================================
  
  // YOUR CODE HERE:
  if (centerSensor >= BLACK_THRESHOLD) {
    Serial.println("BLACK SQUARE DETECTED - Phase 3 Tasks");
    motors.setSpeeds(0, 0);
    delay(500);
    // TODO: Add your Phase 3 pick logic here
  }
  // B) BLUE LINE (between BLUE_MIN_CAL and BLUE_MAX_CAL)
  else if (centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL) {
    Serial.println("BLUE LINE DETECTED - Phase 4 Follow");
    lineFollowing();  //Call the line following function
  }
  //C) FLOOR (off the line)
  else {
    Serial.println("OFF LINE - Phase 1/2 Navigation");
    motors.setSpeeds(0, 0);  // Stop for now
    // TODO: Add your Phase 1/2 wall following logic here
  }
  
  // Small delay to prevent sensor reading too fast
  delay(20);
}
