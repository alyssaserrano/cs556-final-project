#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  /*Initialize values by copying and pasting from PD controller, then declaring for
  the three new variables.*/
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _prevTimeMs = millis();
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _clamp_i = clamp_i;

  _error = 0.0;
  _output = 0.0;
  //_clampOut = 0.0;
  _prevError = 0.0;
  _firstUpdate = true;
  _accumulatedError = 0.0;
}

double PIDcontroller::update(double value, double target_value){
  /*Now copy and paste your PD controller. To implement I component,
  keep track of accumulated error, use your accumulated error in the constrain
  function for the integral, multiply ki by your integral, then add your p, d,
  and i components.
  
  Note: Do not just put all of the integral code at the end of PD component. Think
  about step by step how you can integrate these parts into your PDController
  code.*/
  // Instantiate variables (P + I + D) terms
  float _de = 0.0;
  float d_term = 0.0;
  float i_term = 0.0;
  float p_term = 0.0;


  // 1) Error
  _error = target_value - value;

  // 2) dt in seconds (same as PD controller)
  unsigned long curr_time = millis();
  float dt;

  // In seconds
  dt = (curr_time - _prevTimeMs) / 1000.0;

  // 3) PID output
  // P
  p_term = _kp * _error;

  // I term
  // To get the accumulated error (Error over time) you have to multiply the error by the time
  float _accumulatedError = _accumulatedError + (_error * dt);
  i_term = _ki * constrain(_accumulatedError, -_clamp_i, _clamp_i);

  // D term
  // Check if it is the first update. If so you have to initialize de and d_term = 0.
  if (_firstUpdate == false){
    _de = _error - _prevError;
    d_term = _kd * (_de / dt);
  }
  else {
    _firstUpdate = false;
    _de = 0.0;
    d_term = 0.0;
  }
  
  // output = (K_p * e) + (K_d * (d_e / d_t)) + K_i * accumulatedError
  _output = p_term + i_term + d_term;

  // 6) Final output clamp (actuator limit)
  _output = constrain(_output, _minOutput, _maxOutput);

  // 7) Save state
  _prevError = _error;
  _prevTimeMs = curr_time;

  return _output;
  
}
