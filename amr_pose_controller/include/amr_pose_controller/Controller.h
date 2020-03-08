//
// Created by jakub on 7.3.2020.
//

#ifndef PROJECT_CONTROLLER_H
#define PROJECT_CONTROLLER_H

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>


struct ControllerConfig {
    double linearKp;
    double rotationKp;
    double controllerFrequency;
    double maxLinearSpeed;
    double minLinearSpeed;
    double maxLinearAcceleration;
};


class Controller {
public:
    Controller(const ControllerConfig& config);

    geometry_msgs::Twist getControllerAction(const geometry_msgs::Pose2D& currentPose, const geometry_msgs::Pose2D& requiredPose);

    geometry_msgs::Twist getStopAction();

    double getDistanceError(const geometry_msgs::Pose2D& currentPose, const geometry_msgs::Pose2D& requiredPose);

private:
    double lastLinAction = 0;
    double maxLinSpeed;
    double minLinSpeed;
    double maxLinAcceleration;
    double controllerFrequency;
    double linearKp;
    double rotationKp;

    double getAngleError(const geometry_msgs::Pose2D& currentPose, const geometry_msgs::Pose2D& requiredPose);

};


#endif //PROJECT_CONTROLLER_H
