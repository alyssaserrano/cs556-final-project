#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  _kp = kp;
  _ki = ki;
	_kd = kd;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
  _clamp_i = clamp_i;
  _erracc = 0.0;
	_lastError = 0.0;
	_error = 0.0;
	_output = 0.0;
	_clampOut = 0.0;
	_previousTime = 0.0;
	_currentTime = millis();
}

double PIDcontroller::update(double value, double target_value){
  _error = value - target_value;
  _erracc += _error;
  
  _currentTime = millis();

  double p = _kp*_error;
  double integral = constrain(_erracc*(_currentTime-_previousTime), -_clamp_i, _clamp_i);
  double i = _ki*integral;
  double d;
    
  if (_previousTime!=0.0){
	d = _kd*(_error - _lastError)/(_currentTime-_previousTime);
  }
  else{
	d = 0.0;  
  }

  _output = p + i + d;
  
  _clampOut = constrain(_output, _minOutput, _maxOutput);
  
  _previousTime = _currentTime;
  _lastError = _error;

  return _clampOut;
}
