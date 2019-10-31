#ifndef FLUID_LQR_H
#define FLUID_LQR_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <eigen3/Eigen/Dense>
#include "path.h"

namespace fluid {

    struct NearestIndexResult {
        const unsigned int index;
        const double error;
    };

    struct Result {
        const double angle, x, y, error, error_in_yaw, target_yaw, acceleration;
    };

    class LQR {

        private:

            Eigen::Matrix<double, 5, 5> Q = Eigen::Matrix<double, 5, 5>::Identity(5, 5);
            Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Identity(2, 2);

            Eigen::Matrix<double, 5, 5> solveDARE(const Eigen::Matrix<double, 5, 5>& A, 
                                                  const Eigen::Matrix<double, 5, 2>& B,
                                                  const Eigen::Matrix<double, 5, 5>& Q,
                                                  const Eigen::Matrix<double, 2, 2>& R) const;

            Eigen::Matrix<double, 2, 5> dlqr(const Eigen::Matrix<double, 5, 5>& A, 
                                             const Eigen::Matrix<double, 5, 2>& B,
                                             const Eigen::Matrix<double, 5, 5>& Q,
                                             const Eigen::Matrix<double, 2, 2>& R) const;

        public:

            LQR();

            Result control_law(geometry_msgs::Pose pose, geometry_msgs::Twist twist, fluid::Path path, double previous_error, double previous_error_in_yaw, std::vector<double> speed_profile);
 
    };
}

#endif