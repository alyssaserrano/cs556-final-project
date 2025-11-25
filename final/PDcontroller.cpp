#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
  _kp = kp;
  _kd = kd;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _error = 0.0;
  _output = 0.0;
  _clampOut = 0.0;
  _prevError = 0.0;
  _prevTimeMs = 0;
  _firstUpdate = true;
}

double PDcontroller::update(double value, double target_value){
  //Controller math here
  /*Hints: To add damping (derivative), you must have something to
           keep track of time for the rate of change.
           
           Also note that the first time PD controller is ran, we only have
           the P component, so consider using an if-else statement.

           Again, you need to return actuator controller value (_clampOut)
  */

  // Get the error
  _error = target_value - value;

  // Get the time
  unsigned long time = millis();

  // Instantiate derivative of time for kd
  double dt;

  // Check if time is firstly started because then initilize dt to 0 otherwise get actual derivative of time.
  if(_firstUpdate){
    dt = 0.0;
    _firstUpdate = false;
  }
  else{
    // Get the change of time from the previous ms to now and convert to seconds.
    dt = time - _prevTimeMs / 1000.0;

    // Guard so that you can make sure that you are not dividing by 0 for de/dt.
    if (dt < 1e-6) dt = 1e-6;
  }

  // Derivative of the error
  double dError = (_error - _prevError) / dt;

  // PD output
  _output = (_kp * _error) + (_kd * dError);

  // Clamp
  if (_output > _maxOutput) _clampOut = _maxOutput;
  else if (_output < _minOutput) _clampOut = _minOutput;
  else _clampOut = _output;

  // Save state
  _prevError = _error;
  _prevTimeMs = time;

  return _clampOut;
}
