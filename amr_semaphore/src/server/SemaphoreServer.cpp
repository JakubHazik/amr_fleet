//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/server/SemaphoreServer.h>


SemaphoreServer::SemaphoreServer(ros::NodeHandle& nh) {

}

bool SemaphoreServer::lockNodeCb(amr_msgs::LockNode::Request& req, amr_msgs::LockNode::Response& res) {

    return true;
}

