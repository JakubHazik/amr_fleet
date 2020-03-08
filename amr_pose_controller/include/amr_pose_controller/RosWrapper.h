//
// Created by jakub on 7.3.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <memory>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <amr_pose_controller/Controller.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>

#include <amr_msgs/Point.h>



class RosWrapper {
public:
    RosWrapper(ros::NodeHandle& nh);


private:
    enum class State {
        WAIT_FOR_GOAL,
        PERFORMING_GOAL,
//        ZONE_ACHIEVED,
    };


    State state = State::WAIT_FOR_GOAL;
    ros::Publisher cmdVelPub;
    ros::Subscriber robotPoseSub;
    geometry_msgs::Pose2D robotCurrentPose;
    geometry_msgs::Pose2D requiredGoal;
    double waypointZone;
    double goalZone;

    bool robotPoseReceived = false;
    std::unique_ptr<Controller> controller;

    std::queue<geometry_msgs::Pose2D> waypoints;


    void robotPoseCb(const geometry_msgs::PoseConstPtr& poseMsg);

    void turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg);

    void zoneAchieved();




};


#endif //PROJECT_ROSWRAPPER_H
