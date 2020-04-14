//
// Created by jakub on 8.3.2020.
//

#ifndef PROJECT_TASKMANAGERSERVER_H
#define PROJECT_TASKMANAGERSERVER_H

#include <map>
#include <queue>
#include <yaml-cpp/yaml.h>
#include <tf/transform_datatypes.h>

#include <ros/ros.h>
#include <amr_msgs/Task.h>
#include <amr_msgs/Node.h>
#include <amr_msgs/GetTask.h>
#include <amr_msgs/ResetTask.h>
#include <amr_msgs/DoCustomTaskAsap.h>
#include <amr_msgs/ClientInfo.h>


class Client {
public:
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

        if (!resetTaskSrv.waitForExistence(ros::Duration(5))) {
            ROS_ERROR("Reset service: %s is not available", resetTaskSrv.getService().c_str());
        }

        resetTaskSrv.call(srv);
        if (!srv.response.success) {
            ROS_ERROR("Client %s reset task unsuccessful: %s", clientName.c_str(), srv.response.message.c_str());
        }
        return {srv.response.success, srv.response.message};
    }

    void setClientInfo(const amr_msgs::ClientInfo& clientInfo) {
        amr_msgs::Point pose;
        pose.pose.x = clientInfo.poseWithCovariance.pose.pose.position.x;
        pose.pose.y = clientInfo.poseWithCovariance.pose.pose.position.y;
        pose.pose.theta = tf::getYaw(clientInfo.poseWithCovariance.pose.pose.orientation);
        clientCurrentPose = pose;
        clientInfoReceived = true;
    }

    bool isClientReady() {
        return clientInfoReceived;
    }

    bool getCurrentPose(amr_msgs::Point& currentPose) {
        if (clientInfoReceived) {
            currentPose = clientCurrentPose;
            return true;
        } else {
            return false;
        }
    }

private:
    ros::ServiceClient resetTaskSrv;
    std::string clientName;
    std::list<amr_msgs::Task> tasks;
    amr_msgs::Point clientCurrentPose;
    bool clientInfoReceived = false;
    amr_msgs::Task currentPerformingTask;
};


typedef std::map<std::string, std::shared_ptr<Client>> RobotClients;


class TaskManagerServer {
public:
    TaskManagerServer(ros::NodeHandle& nh);
private:
    ros::ServiceServer getTaskSrvServer;
    ros::ServiceServer doCustomTaskServer;
    ros::ServiceClient planPathSrvClient;
    ros::Subscriber clientInfoSub;

    RobotClients clients;

    bool getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res);
    bool doCustomTaskAsapCb(amr_msgs::DoCustomTaskAsap::Request& req, amr_msgs::DoCustomTaskAsap::Response& res);

    void clientInfoCb(const amr_msgs::ClientInfoConstPtr& msg);

    std::shared_ptr<Client> getClient(const std::string& clientId);

    void parseTasks(const std::string& configFile);

    amr_msgs::Task parseTask(const YAML::Node& taskNode);

};


#endif //PROJECT_TASKMANAGERSERVER_H
