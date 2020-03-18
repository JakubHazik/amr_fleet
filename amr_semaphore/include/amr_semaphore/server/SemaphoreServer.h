//
// Created by jakub on 15.3.2020.
//

#ifndef PROJECT_SEMAPHORESERVER_H
#define PROJECT_SEMAPHORESERVER_H

#include <ros/ros.h>
#include <amr_msgs/LockNode.h>

class SemaphoreServer {

public:
    SemaphoreServer(ros::NodeHandle& nh);

private:
    ros::ServiceServer lockNodeSrv;

    bool lockNodeCb(amr_msgs::LockNode::Request& req, amr_msgs::LockNode::Response& res);

};


#endif //PROJECT_SEMAPHORESERVER_H
