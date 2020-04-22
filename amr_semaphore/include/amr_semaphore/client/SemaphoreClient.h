//
// Created by jakub on 15.3.2020.
//

#ifndef PROJECT_SEMAPHORECLIENT_H
#define PROJECT_SEMAPHORECLIENT_H

#include <string>
#include <ros/ros.h>
#include <amr_msgs/Point.h>
#include <future>

class SemaphoreClient {
public:
    SemaphoreClient(const std::string& lockServiceName);

    virtual ~SemaphoreClient() = default;

    // TODO instead of bool use pair<bool, message>
    bool lockNode(const amr_msgs::Point &node);

    bool unlockNode(const amr_msgs::Point &node);

    std::future<bool> lockNodeAsync(const amr_msgs::Point &node);

    bool unlockAllNodes();

private:
    std::string clientId;
    ros::ServiceClient lockNodeSrv;
};


#endif //PROJECT_SEMAPHORECLIENT_H
