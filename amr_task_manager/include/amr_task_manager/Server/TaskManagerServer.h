//
// Created by jakub on 8.3.2020.
//

#ifndef PROJECT_TASKMANAGERSERVER_H
#define PROJECT_TASKMANAGERSERVER_H

#include <map>
#include <queue>

#include <ros/ros.h>
#include <amr_msgs/Task.h>

#include <amr_msgs/Node.h>
#include <amr_msgs/GetTask.h>
#include <amr_msgs/ResetTask.h>



class Task {
public:
    enum class Commnands {
        PLAN_PATH,
        WAIT_FOR_USER_ACK,
        CHARGE_BATTERY,
        DO_NOTHING
    };

    Commnands command;
    std::pair<amr_msgs::Node, amr_msgs::Node> path;
    double timeout;
};


typedef std::map<std::string, std::queue<Task>> RobotsTasks;


class TaskManagerServer {
public:
    TaskManagerServer(ros::NodeHandle& nh);



private:

    ros::ServiceServer getTaskSrvServer;
    ros::ServiceClient resetSrvServer;
    ros::ServiceClient planPathSrvClient;

    RobotsTasks tasks;


    bool getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res);


    void resetTask(std::string robotId);


};


#endif //PROJECT_TASKMANAGERSERVER_H
