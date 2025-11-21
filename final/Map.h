#include <ArduinoJson.h>
#ifndef MAP_H
#define MAP_H

class Map{
  public:
    Map();
    float closest_distance(float *origin,float theta);
    float* ray_line_intersection(float *ray_origin, float theta, float *point1, float *point2);
    // Track visited states and obstacles
    void visited(int x, int y);
    bool isTraversable(int x, int y);
    bool atGoalLocation(int x, int y);
    bool atDockingStation(int x, int y);
    bool cornerDetected(int x, int y);
    void logWaypoint(int x, int y);
    static const int NUM_X = 9;
    static const int NUM_Y = 4;
    int visitArray[NUM_X][NUM_Y] = {0};
  private:
    // Goal positions
    int goalPositions[3][2] = { {6,2}, {8,0}, {2,1} };
    int dockX = 0;
    int dockY = 0;
};

#endif