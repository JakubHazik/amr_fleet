//
// Created by jakub on 7.3.2020.
//

#ifndef PROJECT_POSECONTROLLER_H
#define PROJECT_POSECONTROLLER_H

#include <memory>
#include <queue>
#include <mutex>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <amr_pose_controller/Controller.h>
#include <tf/transform_listener.h>
#include <amr_semaphore/client/SemaphoreAutomaticClient.h>

// msgs
#include <geometry_msgs/Pose.h>
#include <turtlesim/Pose.h>
#include <amr_msgs/Point.h>
#include <amr_msgs/PerformGoalsAction.h>
#include <amr_semaphore/client/SemaphoreClient.h>
#include <std_srvs/SetBool.h>

#include <memory>
#include <future>

#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rvt = rviz_visual_tools;

typedef actionlib::SimpleActionServer<amr_msgs::PerformGoalsAction> PerformGoalAs;


class PoseController {
public:
    PoseController(ros::NodeHandle& nh);

private:
    enum class State {
        INIT_STATE,
        GET_NEW_GOAL,
        PERFORMING_GOAL,
        NO_POSE_CONTROL,
    };

    // interfaces
    ros::ServiceServer poseControlSwitchSrv;
    ros::Publisher cmdVelPub;
    ros::Publisher currentGoalPub;
    ros::Subscriber robotPoseSub;
    PerformGoalAs performGoalAs;
    tf::TransformListener poseTfListener;

    // poses
    amr_msgs::Point currentRequiredGoal;

    // inernal variables
    std::string tfPrefix;
    State state = State::INIT_STATE;
    std::unique_ptr<Controller> controller;
    std::list<amr_msgs::Point> waypoints;
    double waypointZone;
    bool robotPoseReceived = false;
    std::shared_ptr<SemaphoreAutomaticClient> semaphoreClient;
    std::future<bool> nodeLocked;
    rvt::RvizVisualToolsPtr visual_tools;
    std::mutex waypointsMutex;

    bool poseControlSwitchCb(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void updateRobotPose();

    void acGoalCb();
    void acCancelCb();

    void publishAsFeedback();

    void publishAsResult();

    void visualizeAndPublishCurrentGoal();

    void printWaypoints();
};


#endif //PROJECT_POSECONTROLLER_H
