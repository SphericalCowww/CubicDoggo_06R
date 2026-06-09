#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <algorithm>

namespace cubic_doggo_utils {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PIDController {
public:
    PIDController(double kp, double ki, double kd, double limit, double leak_ratio);
    double update(double error, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double limit_, leak_ratio_;
    double integral_ = 0.0;
    double last_error_ = 0.0;
};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
#endif

