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

class Client {
public:
    explicit Client(const std::string& clientName) {
        ros::NodeHandle nh("/");
        resetSrvServer = nh.serviceClient<amr_msgs::ResetTask>(clientName + "/task_manager_client/reset_task");
    }
    std::queue<Task> tasks;
    ros::ServiceClient resetSrvServer;
};


typedef std::map<std::string, Client> RobotClients;


class TaskManagerServer {
public:
    TaskManagerServer(ros::NodeHandle& nh);



private:

    ros::ServiceServer getTaskSrvServer;
    ros::ServiceClient planPathSrvClient;

    RobotClients clients;


    bool getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res);



};


#endif //PROJECT_TASKMANAGERSERVER_H
