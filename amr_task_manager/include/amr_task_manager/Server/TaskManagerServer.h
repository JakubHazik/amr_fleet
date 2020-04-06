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
#include <amr_msgs/DoCustomTaskAsap.h>


class Client {
public:
    Client() = default;
    explicit Client(const std::string& clientName)
        :   clientName(clientName) {
        ros::NodeHandle nh("/");
        resetTaskSrv = nh.serviceClient<amr_msgs::ResetTask>(clientName + "/task_manager_client/reset_task");
    }

    void doCustomTaskAsap(const amr_msgs::Task& task, bool resumePreviousTask) {
        if (resumePreviousTask) {
            tasks.push_front(currentPerformingTask);
        }

        tasks.push_front(task);
        resetCurrentTask();
    }

    bool isNewTaskAvailable() {
        return !tasks.empty();
    }

    void addNewTask(const amr_msgs::Task& task) {
        tasks.push_back(task);
    }

    amr_msgs::Task getNewTask() {
        currentPerformingTask = tasks.front();
        tasks.pop_front();
        return currentPerformingTask;
    }

    std::pair<bool, std::string> resetCurrentTask() {
        amr_msgs::ResetTask srv;
        resetTaskSrv.call(srv);
        if (!srv.response.success) {
            ROS_ERROR("Client %s reset task: %s", clientName.c_str(), srv.response.message.c_str());
        }
        return {srv.response.success, srv.response.message};
    }

    amr_msgs::Task currentPerformingTask;
private:
    ros::ServiceClient resetTaskSrv;
    std::string clientName;
    std::list<amr_msgs::Task> tasks;
};


typedef std::map<std::string, Client> RobotClients;


class TaskManagerServer {
public:
    TaskManagerServer(ros::NodeHandle& nh);
private:
    ros::ServiceServer getTaskSrvServer;
    ros::ServiceServer pauseClientTask;
    ros::ServiceClient planPathSrvClient;

    RobotClients clients;

    bool getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res);
    bool doCustomTaskAsapCb(amr_msgs::DoCustomTaskAsap::Request& req, amr_msgs::DoCustomTaskAsap::Response& res);

    Client getClient(const std::string& id);
};


#endif //PROJECT_TASKMANAGERSERVER_H
