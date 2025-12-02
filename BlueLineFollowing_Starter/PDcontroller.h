#ifndef PDcontroller_h
#define PDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PDcontroller{
  public:
    PDcontroller(float kp, float kd, double minOutput, double maxOutput);
    double update(double value, double target_value);
    
  private:
    float _kp;
	  float _kd;
    double _maxOutput;
	  double _minOutput;
	  int16_t _lastError;
	  double _error;
	  int16_t _output;
	  double _clampOut;
   long int _previousTime;
   long int _currentTime;
};

#endif
