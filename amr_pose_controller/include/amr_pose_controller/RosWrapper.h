//
// Created by jakub on 7.3.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <memory>
#include <queue>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amr_pose_controller/Controller.h>
#include <tf/transform_listener.h>

// msgs
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <amr_msgs/Point.h>
#include <amr_msgs/PerformGoalsAction.h>


typedef actionlib::SimpleActionServer<amr_msgs::PerformGoalsAction> PerformGoalAs;


class RosWrapper {
public:
    RosWrapper(ros::NodeHandle& nh);

private:
    enum class State {
        WAIT_FOR_GOAL,
        PERFORMING_GOAL,
    };

    // interfaces
    ros::Publisher cmdVelPub;
    ros::Subscriber robotPoseSub;
    PerformGoalAs performGoalAs;
    tf::TransformListener poseTfListener;

    // poses
    geometry_msgs::Pose2D currentRobotPose;
    amr_msgs::Point currentRequiredGoal;

    // inernal variables
    std::string tfPrefix;
    State state = State::WAIT_FOR_GOAL;
    std::unique_ptr<Controller> controller;
    std::queue<amr_msgs::Point> waypoints;
    double waypointZone;
    double goalZone;
    bool robotPoseReceived = false;

    void updateRobotPose();

    void robotPoseCb(const geometry_msgs::PoseConstPtr& poseMsg);

    void turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg);

    void acGoalCb();

    void publishAsFeedback();

    void publishAsResult();
};


#endif //PROJECT_ROSWRAPPER_H
