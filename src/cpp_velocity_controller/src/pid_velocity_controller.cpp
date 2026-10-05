//
// Created by valwe on 2/25/26.
//

#include "cpp_velocity_controller/pid_velocity_controller.hpp"
#include <bits/stdc++.h>

PidVelocityController::PidVelocityController( const double kp,  const double ki,  const double kd,  const double ta,  const double a, int i_vec_length)
: kp_(kp), ki_(ki), kd_(kd), ta_(ta), a_(a), i_vec_length_(i_vec_length){

    current_write_position_ = 0;
    vec_i_ = std::vector<double>(i_vec_length, 0.0);

    last_dist_target_ = 0.0;
}

double PidVelocityController::next_velocity(double dist_target){

    double p, i, d;

    // Low-Pass Filter
    if (first_run_) {
        // prevent massive jump on the first frame
        filtered_dist_target_ = dist_target;
        first_run_ = false;
    } else {
        // Apply EMA
        filtered_dist_target_ = a_ * dist_target + (1.0 - a_) * filtered_dist_target_;
    }

    // Proportional component
    p = kp_ * filtered_dist_target_;

    // Integral component
    vec_i_[current_write_position_] = filtered_dist_target_;
    current_write_position_ = (current_write_position_ + 1) % i_vec_length_;
    i = ki_ * ta_ * std::accumulate(vec_i_.begin(), vec_i_.end(), 0);

    // Derivative component
    d = kd_ / ta_ * (filtered_dist_target_ - last_dist_target_);
    last_dist_target_ = filtered_dist_target_;
    return p + i + d;
}