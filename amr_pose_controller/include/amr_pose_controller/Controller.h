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
    double maxAngularSpeed;
    double minLinearSpeed;
    double maxLinearAcceleration;
    double maxAngularAcceleration;
    double goalDeadZone;
};


class Controller {
public:
    explicit Controller(const ControllerConfig& _config);

    void setCurrentPose(const geometry_msgs::Pose2D& _currentPose);
    void setRequiredPose(const geometry_msgs::Pose2D& _requiredPose);
    const geometry_msgs::Pose2D& getCurrentPose();
    const geometry_msgs::Pose2D& getRequiredPose();

    geometry_msgs::Twist getControllerAction();

    geometry_msgs::Twist getStopAction();

    bool isZoneAchieved(double zoneDistance);

private:
    ControllerConfig config;
    double lastLinAction = 0;
    double lastAngAction = 0;
    geometry_msgs::Pose2D currentPose;
    geometry_msgs::Pose2D requiredPose;

    double getAngleError();

    double getDistanceError();

};


#endif //PROJECT_CONTROLLER_H
