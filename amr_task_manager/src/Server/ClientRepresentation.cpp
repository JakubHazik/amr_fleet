//
// Created by jakub on 14. 4. 2020.
//

#include <amr_task_manager/Server/ClientRepresentation.h>


ClientRepresentation::ClientRepresentation(const std::string &clientName, bool runTasksPeriodically)
        :   clientName(clientName), runTasksPeriodically(runTasksPeriodically) {
    ros::NodeHandle nh("/");
    resetTaskSrv = nh.serviceClient<amr_msgs::ResetTask>(clientName + "/task_manager_client/reset_task");
}

void ClientRepresentation::doCustomTaskAsap(const amr_msgs::Task &task, bool resumePreviousTask) {
    tasksContainerMtx.lock();
    if (resumePreviousTask) {
        tasks.push_front(currentPerformingTask);
    }

    tasks.push_front(task);
    tasksContainerMtx.unlock();
    resetCurrentTask();
}

bool ClientRepresentation::isNewTaskAvailable() {
    return !tasks.empty();
}

void ClientRepresentation::addNewTask(const amr_msgs::Task &task) {
    std::lock_guard<std::mutex> lck(tasksContainerMtx);
    tasks.push_back(task);
}

amr_msgs::Task ClientRepresentation::getNewTask() {
    std::lock_guard<std::mutex> lck(tasksContainerMtx);
    currentPerformingTask = tasks.front();
    tasks.pop_front();

    if (runTasksPeriodically) {
        tasks.push_back(currentPerformingTask);
    }

    return currentPerformingTask;
}

std::pair<bool, std::string> ClientRepresentation::resetCurrentTask() {
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

void ClientRepresentation::setClientInfo(const amr_msgs::ClientInfo &clientInfo) {
    amr_msgs::Point pose;
    pose.pose.x = clientInfo.poseWithCovariance.pose.pose.position.x;
    pose.pose.y = clientInfo.poseWithCovariance.pose.pose.position.y;
    pose.pose.theta = tf::getYaw(clientInfo.poseWithCovariance.pose.pose.orientation);
    clientInfoMtx.lock();
    clientCurrentPose = pose;
    clientInfoMtx.unlock();

    if (newClientInfo) {
        newClientInfo->set_value();
    }
}

void ClientRepresentation::waitForNewClientInfo() {
    newClientInfo = std::make_shared<std::promise<void>>();
    auto fut = newClientInfo->get_future();
    fut.wait();
    newClientInfo.reset();
}

amr_msgs::Point ClientRepresentation::getCurrentPose() {
    std::lock_guard<std::mutex> lck(clientInfoMtx);
    return clientCurrentPose;
}
