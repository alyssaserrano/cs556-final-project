/* 
This code is part of a class Map used for 2D ray-casting calculations on a grid-like environment. 
The Map class includes methods to determine 
the closest distance from 
a point in a given direction (specified by an angle, theta) 
to any line segment within a predefined set of segments (maps array).
*/

#include "Map.h"
#include <math.h>
//DynamicJsonDocument doc(400);
float none[2] = {-1.0,-1.0};
static float res[2]; 

const int PATH_SIZE = 36;

struct Point {
    int x;
    int y;
};

const Point fullPath[PATH_SIZE] = {
    {0,0}, {0,1}, {0,2}, {0,3},
    {1,3}, {2,3}, {3,3}, {4,3}, {5,3}, {5,2}, {5,1}, {6,1}, {6,2},
    {6,3}, {7,3}, {8,3}, {8,2}, {8,1}, {8,0},
    {7,0}, {7,1}, {7,2}, {7,1}, {7,0}, {6,0}, {5,0}, {4,0}, {4,1}, {4,2}, {3,2}, {2,2}, {1,2}, {1,1}, {1,0}, {1,1}, {2,1}
};

// --- WALL SEGMENTS (outer box + stepped inner walls) ---
static const int MAP_N = 9;
float maps[MAP_N][2][2] = {
  // Outer rectangle (CCW)
  { {  0,  0}, {180,  0} },  // bottom
  { {180,  0}, {180, 80} },  // right
  { {180, 80}, {  0, 80} },  // top
  { {  0, 80}, {  0,  0} },  // left

  // Inner blue walls (step shape)
  { {  0, 60}, { 80, 60} },  // long horizontal
  { { 80, 60}, { 80, 40} },  // step down
  { { 80, 40}, {140, 40} },  // horizontal
  { {140, 40}, {140, 60} },  // step up
  { {160, 60}, {160, 80} },  // short vertical near right
};

// Hardcoded goal positions for the map
const int goalPositions[3][2] = { {6,2}, {8,0}, {2,1} };

Map::Map(){}

// Check if at corner location
bool Map::cornerDetected(int x, int y){
  // TODO: Indicate what locations are corners (do this on sim first then implement on hardware.)
  if (x == 1 && y == 3) return true;  // Left side lower corner
  if (x == 1 && y == 1) return true;  // Left side upper corner
  if (x == 4 && y == 2) return true;  // Middle turn
  if (x == 7 && y == 1) return true;  // Right side turn
}

// Check if at goal location
bool Map::atGoalLocation(int x, int y) {
  for (int i = 0; i < 3; ++i) {
    if (goalPositions[i][0] == x && goalPositions[i][1] == y) return true;
  }
  return false;
}

// Check if at docking station
bool Map::atDockingStation(int x, int y) {
  return x == dockX && y == dockY;
}

// Log visited waypoint
void Map::logWaypoint(int x, int y) {
  Serial.print("Visited cell (");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.println(")");
}

// Track Coordinates traveled.
void Map::visited(int x, int y){
  if (isTraversable(x, y)) {
    visitArray[x][y] += 1;     // Mark as visited (count visits)
    logWaypoint(x, y);
    if (atDockingStation(x, y)) {
      Serial.print("At docking station (");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.println(")");
    }
    if (atGoalLocation(x, y)) {
      Serial.print("Reached goal at (");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.println(")");
    }
  }
}

// Obstacle checking
bool Map::isTraversable(int x, int y) {
    // All locations that are not traversable.
    if ((x == 2 && y == 0) || (x == 3 && y == 0)) return false;
    return true;
}


// -------------- PROFESSOR PROVIDED METHODS ---------------
//This method calculates the closest distance 
//from a given origin point in a specified direction theta 
//to any line segment in the maps array.

float Map::closest_distance(float*origin,float theta){
  float result=9999.0;
  for(int i=0;i<8;i++){
    float point1[2] = {maps[i][0][0],maps[i][0][1]};
    float point2[2] = {maps[i][1][0],maps[i][1][1]};
    float *p = ray_line_intersection(origin, theta,point1 , point2);    
    if( p[0] != -1){
        float dist = sqrt(pow(p[0]-origin[0],2)+pow(p[1]-origin[1],2));
        if(dist<result)
          result = dist;
    }      
  }
  return result;
}

//This method calculates the intersection point (if any) between 
//a ray (originating from ray_origin at angle theta) and a line segment defined by point1 and point2.

/* "denominator" calculates the cross product between v2 and v3. 
If denominator is zero, the ray and line segment are parallel, so no intersection occurs.

If the robot uses closest_distance to calculate posterior probabilities in a particle filter,
the return of this function helps assess the likelihood of the particle's position given the map layout.
For example, if a particle's estimated sensor reading (distance to nearest obstacle) 
aligns with the robot's actual sensor reading, 
this particle is likely to be close to the robot’s true position.*/
//Uses dot product where a 2-D cross product is required for the denominator.


float* Map::ray_line_intersection(float ray_origin[2], float theta, float point1[2], float point2[2]){
  float v1[2] = {ray_origin[0] - point1[0] , ray_origin[1] - point1[1]} ;
  float v2[2] = {point2[0] - point1[0] , point2[1] - point1[1]} ;
  float v3[2] = {-sin(theta), cos(theta)};

  float denominator = v2[0]*v3[1] - v2[1]*v3[0]; 
  if (denominator == 0) 
    return none;

  float t1 = (v2[0]*v1[1] - v2[1]*v1[0]) / denominator;         
  float t2 = (v1[0]*v3[1] - v1[1]*v3[0]) / denominator; 

  //If both conditions are met (t1 >= 0.0 and 0.0 <= t2 <= 1.0), 
  ///the intersection point is calculated and returned.
  if (t1 >= 0.0 && 0.0 <= t2 && t2 <= 1.0){
    res[0] = ray_origin[0] + t1 * cos(theta);
    res[1] = ray_origin[1] + t1 * sin(theta);
    return res;
  }

  return none;
  
}
