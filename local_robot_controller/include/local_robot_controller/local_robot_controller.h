//
// Created by jakub on 17.10.2019.
//

#ifndef PROJECT_LOCAL_ROBOT_CONTROLLER_H
#define PROJECT_LOCAL_ROBOT_CONTROLLER_H

#include <vector>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;




class LocalRobotController {
public:
    LocalRobotController(ros::NodeHandle &nh);
    virtual ~LocalRobotController() = default;
    void updatePose();

private:
    ros::NodeHandle nh;
    ros::Subscriber pathSubscriber;
    ros::Subscriber amclPoseSubscriber;
    MoveBaseClient mbClient;

    double waypointZone = 0.3;

    std::vector<geometry_msgs::Pose> requiredPoses;
    geometry_msgs::PoseWithCovariance robotActualPose;
    bool goalIsProcessing;

    void subPathCb(const nav_msgs::Path::ConstPtr& msg);
    void subAmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void sendNewGoal();
    double calculatePosesDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);


};


#endif //PROJECT_LOCAL_ROBOT_CONTROLLER_H
