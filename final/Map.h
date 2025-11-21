#include <ArduinoJson.h>
#ifndef MAP_H
#define MAP_H

class Map{
  public:
    Map();
    float closest_distance(float *origin,float theta);
    float* ray_line_intersection(float *ray_origin, float theta, float *point1, float *point2);
};

#endif