//
// Created by valwe on 2/25/26.
//

#ifndef PID_VELOCITY_CONTROLLER_HPP
#define PID_VELOCITY_CONTROLLER_HPP

#include "cpp_velocity_controller/velocity_controller.hpp"
#include <vector>

/**
 * @class PidVelocityController
 * PID implementation of VelocityController
 * Using EMA filter to smooth input.
 */
class PidVelocityController : public VelocityController {
    public:
        PidVelocityController(const double kp, const double ki, const double kd, const double ta, const double a, const int i_vec_length);

        double next_velocity(double dist_target);

        const double kp_;
        const double ki_;
        const double kd_;
        const double ta_;
        const double a_;
        const int i_vec_length_;
    private:

        int current_write_position_;
        std::vector<double> vec_i_;

        double last_dist_target_;
        double filtered_dist_target_ = 0.0;
        bool first_run_ = true;
};



#endif //PID_VELOCITY_CONTROLLER_HPP
