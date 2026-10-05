//
// Created by valwe on 2/25/26.
//

#ifndef XYZ_VELOCITY_CONTROLLER_HPP
#define XYZ_VELOCITY_CONTROLLER_HPP

#include "cpp_velocity_controller/velocity_controller.hpp"

/**
 * @class XyzVelocityController
 * Composite of VelocityController for x-, y-, and z-axis.
 *
 * Uses a VelocityController per axis.
 */
class XyzVelocityController {
    private:
        std::shared_ptr<VelocityController> ctrl_x_;
        std::shared_ptr<VelocityController> ctrl_y_;
        std::shared_ptr<VelocityController> ctrl_z_;

    public:
        XyzVelocityController(
            std::shared_ptr<VelocityController> x,
            std::shared_ptr<VelocityController> y,
            std::shared_ptr<VelocityController> z
        ) : ctrl_x_(std::move(x)),
            ctrl_y_(std::move(y)),
            ctrl_z_(std::move(z)) {}

        [[nodiscard]] const std::shared_ptr<VelocityController>& get_ctrl_x() const { return ctrl_x_; }
        [[nodiscard]] const std::shared_ptr<VelocityController>& get_ctrl_y() const { return ctrl_y_; }
        [[nodiscard]] const std::shared_ptr<VelocityController>& get_ctrl_z() const { return ctrl_z_; }

        // X-axis velocity function
        double next_velocity_x(double dist_target) {
            if (ctrl_x_) {
                return ctrl_x_->next_velocity(dist_target);
            }
            return dist_target; // Fallback if pointer is null
        }

        // Y-axis velocity function
        double next_velocity_y(double dist_target) {
            if (ctrl_y_) {
                return ctrl_y_->next_velocity(dist_target);
            }
            return dist_target; // Fallback if pointer is null
        }

        // Z-axis velocity function
        double next_velocity_z(double dist_target) {
            if (ctrl_z_) {
                return ctrl_z_->next_velocity(dist_target);
            }
            return dist_target; // Fallback if pointer is null
        }
};



#endif //XYZ_VELOCITY_CONTROLLER_HPP
