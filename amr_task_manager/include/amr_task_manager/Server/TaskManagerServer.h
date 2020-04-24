//
// Created by jakub on 8.3.2020.
//

#ifndef PROJECT_TASKMANAGERSERVER_H
#define PROJECT_TASKMANAGERSERVER_H

#include <map>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <amr_msgs/Task.h>
#include <amr_msgs/Node.h>
#include <amr_msgs/GetTask.h>
#include <amr_msgs/ResetTask.h>
#include <amr_msgs/DoCustomTaskAsap.h>
#include <amr_msgs/ClientInfo.h>

#include <amr_task_manager/Server/ClientRepresentation.h>

typedef std::map<std::string, std::shared_ptr<ClientRepresentation>> RobotClients;


class TaskManagerServer {
public:
    TaskManagerServer();
private:
    ros::ServiceServer getTaskSrvServer;
    ros::ServiceServer doCustomTaskServer;
    ros::ServiceClient planPathSrvClient;
    ros::Publisher clientPathsPub;
    ros::Subscriber clientInfoSub;

    std::atomic_int numRunningSrvCallbacks;
    const int maxRunningSrvCallbacks = 10;
    RobotClients clients;

    bool getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res);
    bool doCustomTaskAsapCb(amr_msgs::DoCustomTaskAsap::Request& req, amr_msgs::DoCustomTaskAsap::Response& res);

    void clientInfoCb(const amr_msgs::ClientInfoConstPtr& msg);

    std::shared_ptr<ClientRepresentation> getClient(const std::string& clientId);

    void parseTasks(const std::string& configFile);

    amr_msgs::Task parseTask(const YAML::Node& taskNode);

};


#endif //PROJECT_TASKMANAGERSERVER_H
