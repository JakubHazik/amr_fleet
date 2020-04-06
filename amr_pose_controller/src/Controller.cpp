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


Controller::Controller(const ControllerConfig& _config) {
    this->config = _config;
}

void Controller::setCurrentPose(const geometry_msgs::Pose2D& _currentPose) {
    this->currentPose = _currentPose;
}

void Controller::setRequiredPose(const geometry_msgs::Pose2D& _requiredPose) {
    this->requiredPose = _requiredPose;
}

const geometry_msgs::Pose2D& Controller::getCurrentPose() {
    return this->currentPose;
}

const geometry_msgs::Pose2D& Controller::getRequiredPose() {
    return this->requiredPose;
}

geometry_msgs::Twist Controller::getControllerAction() {
    // check goal dead zone
    if (isZoneAchieved(config.goalDeadZone)) {
        return getStopAction();
    }

    // angle action
    double angularAction = config.rotationKp * getAngleError();
    // ramp angular speed up
    double cycleMaxAngSpeed = lastAngAction + config.maxAngularAcceleration / config.controllerFrequency;
    if (std::abs(angularAction) > cycleMaxAngSpeed) {
        angularAction = cycleMaxAngSpeed * signum(angularAction);
    }
    // angular saturation
    if (angularAction > config.maxAngularSpeed) {
        angularAction = config.maxAngularSpeed;
    } else if (angularAction < -config.maxAngularSpeed) {
        angularAction = -config.maxAngularSpeed;
    }

    // linear action
    double linearAction = getDistanceError() * config.linearKp;
    // ramp linear speed up
    double cycleMaxLinSpeed = lastLinAction + config.maxLinearAcceleration / config.controllerFrequency;
    if (linearAction > cycleMaxLinSpeed) {
        linearAction = cycleMaxLinSpeed;
    }
    linearAction = linearAction - std::abs(angularAction) * 0.03;  // slowdown linear if angular error is huge

    // linear saturation
    if (linearAction > config.maxLinearSpeed) {
        linearAction = config.maxLinearSpeed;
    } else if (linearAction < config.minLinearSpeed) {
        linearAction = config.minLinearSpeed;
    }

//    output = output * pow(error/MAX_SPEED,0.3);

    lastLinAction = linearAction;
    lastAngAction = std::abs(angularAction);

    geometry_msgs::Twist action;
    action.angular.z = angularAction;
    action.linear.x = linearAction;

//    ROS_INFO("Error: Angle = %f; Linear = %f", getAngleError() * RAD2DEG, getDistanceError());
//    ROS_INFO("Action: Angle = %f; Linear= %f; Compensation = %f",
//            angularAction, linearAction, - std::abs(angularAction) * 0.05);

    return action;
}


double Controller::getAngleError() {

    double goalAngle = atan2(requiredPose.y - currentPose.y, requiredPose.x - currentPose.x);
    double robotAngle = currentPose.theta;
    double result;

    if (signum(robotAngle) == signum(goalAngle)) {
        // angles are in the one half plane
        return goalAngle - robotAngle;
    }

    if (robotAngle > 0) {
        result = -1 * (std::abs(goalAngle) + std::abs(robotAngle));
        if (result < -M_PI) {
            result = 2 * M_PI + result;
        }
    } else {
        result = std::abs(goalAngle) + std::abs(robotAngle);
        if (result > M_PI) {
            result = -2 * M_PI + result;
        }
    }

    return result;
}


double Controller::getDistanceError() {

    return sqrt(sqr(currentPose.x - requiredPose.x) + sqr(currentPose.y - requiredPose.y));
}

geometry_msgs::Twist Controller::getStopAction() {
    // todo ramp speed down
    return { };
}

bool Controller::isZoneAchieved(double zoneDistance) {
    return zoneDistance > getDistanceError();
}

