//
// Created by jakub on 15.3.2020.
//

#include <amr_semaphore/client/SemaphoreClient.h>
#include <amr_msgs/LockPoint.h>

SemaphoreClient::SemaphoreClient(const std::string &lockServiceName) {
    ros::NodeHandle nh;
    lockNodeSrv = nh.serviceClient<amr_msgs::LockPoint>(lockServiceName);
    if (!lockNodeSrv.waitForExistence(ros::Duration(5))) {
        ROS_ERROR("Unable to contact semaphore_server service: %s", lockServiceName.c_str());
        ros::shutdown();
    }

    clientId = ros::this_node::getNamespace();
}

bool SemaphoreClient::lockNode(const amr_msgs::Point &node) {
    amr_msgs::LockPoint srv;
    srv.request.clientId = clientId;
    srv.request.point = node;

    lockNodeSrv.call(srv);

    if (srv.response.success) {
        return true;
    } else {
        ROS_WARN("Unable to lock node: %s", srv.response.message.c_str());
        return false;
    }
}

//bool SemaphoreClient::unlockNode(const amr_msgs::Node &node) {
//    amr_msgs::LockNode srv;
//    srv.request.clientId = clientId;
//    srv.request.node = node;
//    srv.request.lock = false;
//
//    lockNodeSrv.call(srv);
//
//    if (srv.response.success) {
//        return true;
//    } else {
//        ROS_WARN("%s", srv.response.message.c_str());
//        return false;
//    }
//}

std::future<bool> SemaphoreClient::lockNodeAsync(const amr_msgs::Point &node) {
    return std::async(std::launch::async, &SemaphoreClient::lockNode, this, node);
}

bool SemaphoreClient::unlockAllNodes() {
    amr_msgs::LockPoint srv;
    srv.request.clientId = clientId;
    srv.request.unlockAll = true;
    lockNodeSrv.call(srv);

    if (srv.response.success) {
        return true;
    } else {
        ROS_WARN("%s", srv.response.message.c_str());
        return false;
    }
}
