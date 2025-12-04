#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller {
public:
  PIDcontroller(float kp, float ki, float kd,
                double minOutput, double maxOutput,
                double clamp_i);

  double update(double value, double target_value);

private:
  // Gains
  float _kp, _ki, _kd;

  // Output limits
  double _minOutput, _maxOutput;

  // State
  double _error;
  double _output;
  double _prevError;
  unsigned long _prevTimeMs;
  bool _firstUpdate;

  // Integral (I) state
  double _accumulatedError;   // <— matches your constructor initialization
  double _clamp_i;            // anti-windup clamp for I term
};

#endif
