//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/Controller.h>
#include <cmath>
#include <ros/ros.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

// return signum:  -1, 0, 1
template <typename T>
int signum(T val) {
    return (T(0) < val) - (val < T(0));
}

template <typename T>
constexpr T sqr(T x) {
    return (x) * (x);
}


Controller::Controller(const ControllerConfig& config) {
    maxLinSpeed = config.maxLinearSpeed;
    minLinSpeed = config.minLinearSpeed;
    maxLinAcceleration = config.maxLinearAcceleration;
    controllerFrequency = config.controllerFrequency;
    rotationKp = config.rotationKp;
    linearKp = config.linearKp;
}

geometry_msgs::Twist Controller::getControllerAction(const geometry_msgs::Pose2D& currentPose,
                                                     const geometry_msgs::Pose2D& requiredPose) {
    geometry_msgs::Twist action;

    // angle action
    action.angular.z = rotationKp * getAngleError(currentPose, requiredPose);

    //linear action
    double linearError = getDistanceError(currentPose, requiredPose);
    double linearAction = linearError * linearKp;

    // saturation
    if (linearAction > maxLinSpeed) {
        linearAction = maxLinSpeed;

        // ramp speed up
        if (lastLinAction < linearAction) {
            linearAction = lastLinAction + maxLinAcceleration / controllerFrequency;
            if (linearAction < minLinSpeed) {
                linearAction = minLinSpeed;
            }
        }
    }

//    output = output * pow(error/MAX_SPEED,0.3);

    lastLinAction = linearAction;

    action.linear.x = linearAction;

    ROS_DEBUG("Error: Angle = %f; Linear = %f",
            getAngleError(currentPose, requiredPose) * RAD2DEG, getDistanceError(currentPose, requiredPose));

    return action;
}


double Controller::getAngleError(const geometry_msgs::Pose2D& currentPose,
                                 const geometry_msgs::Pose2D& requiredPose) {

    double goalAngle = atan2(requiredPose.y - currentPose.y, requiredPose.x - currentPose.x);
    double robotAngle = currentPose.theta;
    double result;

    if (signum(robotAngle) == signum(goalAngle)) {
        // angles are in the one half plane
        return goalAngle - robotAngle;
    }

    if (robotAngle > 0) {
        result = -1 * (std::abs(goalAngle) + std::abs(robotAngle));
    } else {
        result = std::abs(goalAngle) + std::abs(robotAngle);
    }

    if (result > M_PI) {
        result = -2 * M_PI + std::abs(robotAngle) + std::abs(goalAngle);
    }

    if (result < -M_PI) {
        result = 2 * M_PI - std::abs(robotAngle) - std::abs(goalAngle);
    }

    return result;
}


double Controller::getDistanceError(const geometry_msgs::Pose2D& currentPose,
                                    const geometry_msgs::Pose2D& requiredPose) {

    return sqrt(sqr(currentPose.x - requiredPose.x) + sqr(currentPose.y - requiredPose.y));
}

geometry_msgs::Twist Controller::getStopAction() {
    return { };
}
