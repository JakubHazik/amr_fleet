//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/client/SemaphoreClient.h>
#include <amr_msgs/LockPoint.h>
#include <amr_msgs/SetupSemaphore.h>

SemaphoreClient::SemaphoreClient(const std::string &lockServiceName, const std::string& setupServiceName, unsigned int nodesNum) {
    ros::NodeHandle nh;
    clientId = ros::this_node::getNamespace();
    // remove slash from client namespace
    if (!clientId.empty() && clientId[0] == '/') {
        clientId.erase(0, 1);
    }

    setupSemaphoreSrv = nh.serviceClient<amr_msgs::SetupSemaphore>(setupServiceName);
    if (!setupSemaphoreSrv.waitForExistence(ros::Duration(5))) {
        ROS_ERROR("Unable to contact semaphore_server service: %s", setupServiceName.c_str());
        ros::shutdown();
    }
    amr_msgs::SetupSemaphore srv;
    srv.request.clientId = clientId;
    srv.request.numReallyLockedNodes = nodesNum;
    if (!setupSemaphoreSrv.call(srv) || !srv.response.success) {
        ROS_ERROR("Error occurred while service: %s has been called", setupServiceName.c_str());
        ros::shutdown();
    }

    lockNodeSrv = nh.serviceClient<amr_msgs::LockPoint>(lockServiceName);
}

bool SemaphoreClient::lockNode(const amr_msgs::Point &node) {
    amr_msgs::LockPoint srv;
    srv.request.clientId = clientId;
    srv.request.point = node;
    srv.request.lock = true;

    if (!lockNodeSrv.call(srv)) {
        ROS_ERROR("Unable to call lock_node service");
        return false;
    }

    if (srv.response.success) {
        return true;
    } else {
//        ROS_WARN("Unable to lock node: %s", srv.response.message.c_str());
        return false;
    }
}

bool SemaphoreClient::unlockNode(const amr_msgs::Point& node) {
    amr_msgs::LockPoint srv;
    srv.request.clientId = clientId;
    srv.request.point = node;
    srv.request.lock = false;

    if (!lockNodeSrv.call(srv)) {
        ROS_ERROR("Unable to call lock_node service");
        return false;
    }

    if (srv.response.success) {
        return true;
    } else {
        ROS_WARN("%s", srv.response.message.c_str());
        return false;
    }
}

std::future<bool> SemaphoreClient::lockNodeAsync(const amr_msgs::Point &node) {
    return std::async(std::launch::async, &SemaphoreClient::lockNode, this, node);
}

bool SemaphoreClient::unlockAllNodes() {
    amr_msgs::LockPoint srv;
    srv.request.clientId = clientId;
    srv.request.unlockAll = true;

    if (!lockNodeSrv.call(srv)) {
        ROS_ERROR("Unable to call lock_node service");
        return false;
    }

    if (srv.response.success) {
        return true;
    } else {
        ROS_WARN("%s", srv.response.message.c_str());
        return false;
    }
}
