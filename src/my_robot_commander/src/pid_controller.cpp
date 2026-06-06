#include "my_robot_commander/pid_controller.hpp"

namespace cubic_doggo_utils {
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PIDController::PIDController(double kp, double ki, double kd, double limit)
    : kp_(kp), ki_(ki), kd_(kd), limit_(limit) {}

double PIDController::update(double error, double dt) {
    if (dt <= 0.0) return 0.0;
    integral_ = 0.98*integral_ + error*dt;              // leaky integrator, 1/20 = 0.98 update step size

    double p_term = kp_*error;
    double i_term = ki_*integral_;
    double d_term = kd_*(error - last_error_)/dt;
    double output = p_term + i_term + d_term;
    
    last_error_ = error;
    return std::max(-limit_, std::min(limit_, output));
}
void PIDController::reset() {
    integral_   = 0.0;
    last_error_ = 0.0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}
