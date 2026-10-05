//
// Created by valwe on 2/25/26.
//

#ifndef VELOCITY_CONTROLLER_HPP
#define VELOCITY_CONTROLLER_HPP

/**
 * @class VelocityController
 * Base class to get velocity from distance from target.
 */
class VelocityController {
    public:
        /**
         * Shall be overridden to implement different types of controllers.
         * @param dist_target Distance from target.
         * @return Velocity towards the target.
         */
        virtual double next_velocity(double dist_target) = 0;
};



#endif //VELOCITY_CONTROLLER_HPP
