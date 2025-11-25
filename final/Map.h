#include <ArduinoJson.h>
#ifndef MAP_H
#define MAP_H

struct Point { int x; int y; };
extern const int PATH_SIZE;     
extern const Point fullPath[];

enum Mode { MOVE_FORWARD, TURN_LEFT, TURN_RIGHT, AT_GOAL, REVERSE, STOP, NONE };

class Map{
  public:
    Map();
    // Default lab functions
    float closest_distance(float *origin,float theta);
    float* ray_line_intersection(float *ray_origin, float theta, float *point1, float *point2);
    
    // Track visited states and obstacles
    void visited(int x, int y);
    bool isTraversable(int x, int y);
    bool atGoalLocation(int x, int y);
    bool atDockingStation(int x, int y);
    Mode cornerDetected(int x, int y);
    void logWaypoint(int x, int y);

    // Instantiate to zero for visit array of coordinates
    static const int MAP_WIDTH = 9;
    static const int MAP_HEIGHT = 4;
    int visitArray[MAP_WIDTH][MAP_HEIGHT] = {0};
  private:
    // Goal positions
    int goalPositions[3][2] = { {6,2}, {8,0}, {2,1} };
    int dockX = 0;
    int dockY = 0;
};

#endif
