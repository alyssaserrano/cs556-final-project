#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  _kp = kp;
	_kd = kd;
  _maxOutput = maxOutput;
	_minOutput = minOutput;
	_lastError = 0;
	_error = 0;
	_output = 0.0;
	_clampOut = 0.0;
	_previousTime = 0.0;
	_currentTime = millis();
}

double PDcontroller::update(double value, double target_value){
  _error = value - target_value;

  _currentTime = millis();

  if (_previousTime!=0.0){
    	_output = _kp*_error  + _kd*(_error - _lastError)/(_currentTime-_previousTime);
  }
  else{
	_output = _kp*_error;  
  }
  // _output = _kp*_error  + _kd*(_error - _lastError);
  
  _lastError = _error;
  
  _clampOut = constrain(_output, _minOutput, _maxOutput);

  return _clampOut;
}
