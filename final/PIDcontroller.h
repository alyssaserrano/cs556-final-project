#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i);
    double update(double value, double target_value);
    
  private:
    float _kp;
    float _ki;
	  float _kd;
    double _maxOutput;
	  double _minOutput;
    double _clamp_i;
    double _erracc;
	  double _lastError;
	  double _error;
	  double _output;
	  double _clampOut;
    long int _previousTime;
    long int _currentTime;
};

#endif
