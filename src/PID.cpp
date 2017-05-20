#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  // all the errors are initialized as zero
  _p_error = 0.0;
  _i_error = 0.0;
  _d_error = 0.0;

  _Kp = Kp; // Proportional gain (applied to the current error)
  _Ki = Ki; // Integral gain (applied to the sum of all errors since the start)
  _Kd = Kd; // Differential gain (applied to the difference between current error and previous one)
}

void PID::UpdateError(double cte) {

    // 1st update the differential error
    _d_error = cte - _p_error;

    // update the integral error
    _i_error = _i_error + cte;

    // update the proportional error
    _p_error = cte;

}

double PID::TotalError() {
// Compute the total error by applying the gains to the corresponding error
    return - _Kp * _p_error - _Ki * _i_error - _Kd * _d_error;
}
