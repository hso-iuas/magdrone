//
// Created by valwe on 2/17/26.
//

#ifndef OFFSET_TO_TARGET_HPP
#define OFFSET_TO_TARGET_HPP

#include <string>
#include <sstream>

/**
 * @class OffsetToTarget
 * @brief Simple class to store offset information.
 */
class OffsetToTarget {
  public:
    double x_;
    double y_;
    double z_;
    float yaw_;
    float yaw_rate_;

    OffsetToTarget(double x, double y, double z, float yaw, float yaw_rate) :
        x_(x), y_(y), z_(z), yaw_(yaw), yaw_rate_(yaw_rate) {}

    std::string to_string() const {
        std::ostringstream oss;
        oss << "v["<< x_
            << ", " << y_
            << ", " << z_
            << ", " << yaw_
            << ", " << yaw_rate_ << "]";
        return oss.str();
    }
};



#endif //OFFSET_TO_TARGET_HPP
