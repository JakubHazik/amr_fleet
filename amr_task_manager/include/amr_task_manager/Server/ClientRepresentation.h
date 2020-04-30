//
// Created by jakub on 14. 4. 2020.
//

#ifndef SRC_CLIENTREPRESENTATION_H
#define SRC_CLIENTREPRESENTATION_H

#include <ros/ros.h>
#include <list>
#include <future>
#include <mutex>
#include <amr_msgs/ResetTask.h>
#include <amr_msgs/DoCustomTaskAsap.h>
#include <amr_msgs/ClientInfo.h>
#include <tf/transform_datatypes.h>

class ClientRepresentation {
public:
    explicit ClientRepresentation(const std::string& clientName, bool runTasksPeriodically);

    void doCustomTaskAsap(const amr_msgs::Task& task, bool resumePreviousTask);

    bool isNewTaskAvailable();

    void addNewTask(const amr_msgs::Task& task);

    amr_msgs::Task getNewTask();

    std::pair<bool, std::string> resetCurrentTask();

    void setClientInfo(const amr_msgs::ClientInfo& clientInfo);

    void waitForNewClientInfo();

    amr_msgs::Point getCurrentPose();

private:
    ros::ServiceClient resetTaskSrv;
    std::string clientName;
    std::list<amr_msgs::Task> tasks;
    amr_msgs::Point clientCurrentPose;
    amr_msgs::Task currentPerformingTask;
    std::shared_ptr<std::promise<void>> newClientInfo;

    std::mutex tasksContainerMtx;
    std::mutex clientInfoMtx;
    bool runTasksPeriodically;
};


#endif //SRC_CLIENTREPRESENTATION_H
