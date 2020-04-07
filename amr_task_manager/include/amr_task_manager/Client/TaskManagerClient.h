//
// Created by jakub on 8.3.2020.
//

#ifndef PROJECT_TASKMANAGERCLIENT_H
#define PROJECT_TASKMANAGERCLIENT_H

#include <ros/ros.h>
#include <amr_msgs/GetTask.h>
#include <amr_msgs/ResetTask.h>
#include <actionlib/client/simple_action_client.h>
#include <amr_msgs/PerformGoalsAction.h>


typedef actionlib::SimpleActionClient<amr_msgs::PerformGoalsAction> PerformWaypoints;


class TaskManagerClient {
public:
    TaskManagerClient(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh;
    ros::ServiceClient getTaskSrvClient;
    ros::ServiceClient enablePoseControlClient;
    ros::ServiceServer resetTaskSrvServer;
    ros::Publisher currentTaskPub;
    PerformWaypoints performWaypointsAc;
    ros::Timer getNewTaskTimer;
    std::string clientId;
    amr_msgs::Task currentTask;

    // callbacks
    bool resetTaskServiceCb(amr_msgs::ResetTask::Request& req, amr_msgs::ResetTask::Response& res);

    void acWaypointsDoneCb(const actionlib::SimpleClientGoalState& state,
                           const amr_msgs::PerformGoalsResultConstPtr& result);

    void acWaypointsFeedbackCb(const amr_msgs::PerformGoalsFeedbackConstPtr& feedback);

    void activeCallback();


    // other methods
    bool getNewTask();

    bool performTask(amr_msgs::Task& task);

    void callGetNewTaskServiceAfterTime(double timeout);

};


#endif //PROJECT_TASKMANAGERCLIENT_H
