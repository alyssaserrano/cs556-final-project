#ifndef Particle_Filter_h
#define Particle_Filter_h
#include <Pololu3piPlus32U4.h>
#include "Map.h"
#include "sonar.h"
using namespace Pololu3piPlus32U4;

const uint8_t MaxParticleNumber = 25; 

class ParticleFilter{
  public:
    ParticleFilter(const Map& map, int lenOfMap, int num_particles, float translation_variance, float rotation_variance, float measurement_variance);
    void move_particles(float x, float y, float theta);
    void measure();
    void resample(float maxprob);
    void estimate_position();
    void print_particles();
    
    // Getter methods for confidence checking
    float get_particle_x(uint8_t index);
    float get_particle_y(uint8_t index);
    float get_particle_probability(uint8_t index);
    
    struct Particle{
      public:
          float x;
          float y;
          float angle;
          float probability;
      private:
    };
    
  private:
    int _lenOfMap;
    float _translation_variance;
    float _rotation_variance;
    float _measurement_variance;
    int _iter;

    const Map* _mp; 
    float _x_est;
    float _y_est;
    float _angle_est;
    
    int _num_particles;
    Sonar _sonar = Sonar(4);
    ParticleFilter::Particle _particle_list[25];
};

#endif
