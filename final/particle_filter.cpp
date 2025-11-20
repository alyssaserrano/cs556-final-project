#include <Pololu3piPlus32U4.h>
#include <Gaussian.h>
#include "particle_filter.h"
#include "sonar.h"
#include <math.h>

using namespace Pololu3piPlus32U4;

static float wrap_pi(float a){ while(a<=-PI) a+=2*PI; while(a>PI) a-=2*PI; return a; }
static float clamp(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }

ParticleFilter::ParticleFilter(const Map& map, int lenOfMap, int num_particles, float translation_variance, float rotation_variance, float measurement_variance) {
  _num_particles = num_particles;
  _translation_variance = translation_variance;
  _rotation_variance = rotation_variance;
  _measurement_variance = measurement_variance;
  _lenOfMap = lenOfMap;
  _iter = 0;

  _x_est = 0;
  _y_est = 0;
  _angle_est = 0;
  _mp = &map;

  const float invN = 1.0f / (float)_num_particles;
  for(uint8_t i=0;i<_num_particles; i++){
    float rx = (float)random(0, 1700) / 10000.0f;
    float ry = (float)random(0, 1700) / 10000.0f;
    float ra = (float)random(0, 1700) / 10000.0f;

    _particle_list[i].x = rx * _lenOfMap;
    _particle_list[i].y = ry * _lenOfMap;
    _particle_list[i].angle = (ra * 2.0f * PI) - PI;
    _particle_list[i].probability = invN;
  }
}

void ParticleFilter::move_particles(float dx, float dy, float dtheta){
  Gaussian gTrans(0.0f, _translation_variance);
  Gaussian gRot(0.0f, _rotation_variance);

  for (uint8_t i = 0; i < _num_particles; i++) {
    float n_fwd = gTrans.random();
    float n_lat = gTrans.random();
    float n_yaw = gRot.random();

    float a = _particle_list[i].angle;

    float dx_local = dx + n_fwd;
    float dy_local = dy + n_lat;

    float dx_world = dx_local * cosf(a) - dy_local * sinf(a);
    float dy_world = dx_local * sinf(a) + dy_local * cosf(a);
    
    _particle_list[i].x += dx_world;
    _particle_list[i].y += dy_world;
    _particle_list[i].angle += dtheta + n_yaw;
    
    _particle_list[i].angle = wrap_pi(_particle_list[i].angle);
    _particle_list[i].x = clamp(_particle_list[i].x, 0.0f, (float)_lenOfMap);
    _particle_list[i].y = clamp(_particle_list[i].y, 0.0f, (float)_lenOfMap);
  }
}

void ParticleFilter::measure(){
  const float grid_scale_cm = (_lenOfMap / 3.0f);
  const float z_meas = _sonar.readDist();
  float maxlog = -1e30f;

  for(uint8_t i = 0; i < _num_particles; i++){
    float origin[2] = {
      _particle_list[i].x / grid_scale_cm,
      _particle_list[i].y / grid_scale_cm
    };
    float particledist = _mp->closest_distance(origin, _particle_list[i].angle);
    float expected_cm = particledist * grid_scale_cm;
    float z = z_meas;
    float s2 = _measurement_variance;
    float loglik = -0.5f * ((z - expected_cm)*(z - expected_cm)/s2 + logf(2.0f * PI * s2));

    _particle_list[i].probability = logf(fmaxf(1e-30f, _particle_list[i].probability)) + loglik;

    if (_particle_list[i].probability > maxlog) {
      maxlog = _particle_list[i].probability;
    }
  }

  float norm_factor = 0.0f;
  for(uint8_t i = 0; i < _num_particles; i++){
    _particle_list[i].probability = expf(_particle_list[i].probability - maxlog);
    norm_factor += _particle_list[i].probability;
  }

  float maxprob = 0.0f;
  if (norm_factor <= 1e-12f) {
    const float uniform = 1.0f / (float)_num_particles;
    for (uint8_t i = 0; i < _num_particles; i++) {
      _particle_list[i].probability = uniform;
    }
    return;
  } else {
    for(uint8_t i = 0; i < _num_particles; i++){
      _particle_list[i].probability /= norm_factor;
      if (maxprob < _particle_list[i].probability) {
        maxprob = _particle_list[i].probability;
      }
    }
  }

  resample(maxprob);
}

void ParticleFilter::resample(float maxprob){
  float b = 0.0;
  float norm_tot = 0;
  uint8_t index = (int)random(_num_particles);
  Particle temp_particles[_num_particles];

  for(uint8_t i=0; i<_num_particles; i++){
    b += ((float)(random(100))/100) * 2.0 * maxprob;
    while(b > _particle_list[index].probability){
      b -= _particle_list[index].probability;
      index = (index+1) % _num_particles; 
    }
    temp_particles[i].x = _particle_list[index].x;
    temp_particles[i].y = _particle_list[index].y;
    temp_particles[i].angle = _particle_list[index].angle;
    temp_particles[i].probability = _particle_list[index].probability;
    norm_tot += temp_particles[i].probability;
  }

  for(uint8_t i=0; i<_num_particles; i++){
    temp_particles[i].probability /= norm_tot;
  }  

  for(uint8_t i=0; i<_num_particles; i++){
    _particle_list[i] = temp_particles[i];
  }
}

void ParticleFilter::print_particles(){
  _iter++;
  estimate_position();

  Serial.println("========== Particles ==========");
  for (uint8_t i = 0; i < _num_particles; i++) {
    Serial.print("  p"); Serial.print(i);
    Serial.print(": x="); Serial.print(_particle_list[i].x, 2);
    Serial.print(" y="); Serial.print(_particle_list[i].y, 2);
    Serial.print(" th="); Serial.print(_particle_list[i].angle, 3);
    Serial.print(" w="); Serial.println(_particle_list[i].probability, 4);
  }
  Serial.print("Iter="); Serial.print(_iter);
  Serial.print(" | Estimate: ("); 
  Serial.print(_x_est, 2); Serial.print(", ");
  Serial.print(_y_est, 2); Serial.print(") th=");
  Serial.println(_angle_est, 3);
  Serial.println("===============================");
}

void ParticleFilter::estimate_position(){
  _x_est = 0.0;
  _y_est = 0.0;
  float cos_sum = 0.0;
  float sin_sum = 0.0;

  for (uint8_t i = 0; i < _num_particles; i++) {
    float w = _particle_list[i].probability;
    _x_est += w * _particle_list[i].x;
    _y_est += w * _particle_list[i].y;
    cos_sum += w * cosf(_particle_list[i].angle);
    sin_sum += w * sinf(_particle_list[i].angle);
  }

  _angle_est = atan2f(sin_sum, cos_sum);
}

// Getter methods for confidence checking
float ParticleFilter::get_particle_x(uint8_t index) {
  if (index < _num_particles) {
    return _particle_list[index].x;
  }
  return 0.0;
}

float ParticleFilter::get_particle_y(uint8_t index) {
  if (index < _num_particles) {
    return _particle_list[index].y;
  }
  return 0.0;
}

float ParticleFilter::get_particle_probability(uint8_t index) {
  if (index < _num_particles) {
    return _particle_list[index].probability;
  }
  return 0.0;
}
