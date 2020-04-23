//
// Created by jakub on 23. 4. 2020.
//

#include <stdio.h>      /* puts */
#include <time.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <amr_msgs/ClientInfo.h>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

ros::Subscriber monitorSub;
std::string logDir;

void monitorDataCb(const amr_msgs::ClientInfoPtr& msg) {
    std::ofstream clientFile;
    std::string filepath = logDir + "/" + msg->clientId + ".csv";

    // write header
    if (!fs::exists(filepath)) {
        clientFile.open(filepath, std::ios_base::app);
        clientFile << "Time; X; Y; GoalNodeUUID; TaskHash;" << std::endl;
        clientFile.close();
    }

    clientFile.open(filepath, std::ios_base::app);
    clientFile << ros::Time::now() << ";";
    clientFile << msg->poseWithCovariance.pose.pose.position.x << "; ";
    clientFile << msg->poseWithCovariance.pose.pose.position.y << "; ";
    clientFile << msg->robotCurrentGoal.uuid << "; ";
    clientFile << msg->currentTask.taskUuid << "; ";
    clientFile << std::endl;
    clientFile.close();
}

std::string getTimestamp() {
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time (&rawtime);
    timeinfo = localtime (&rawtime);

    strftime(buffer,80,"%Y_%m_%d_%H_%M", timeinfo);
    return {buffer};
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "monitor_logger" );
    ros::NodeHandle nh;

    // create dir
    auto directoryPath = fs::path(getenv("HOME")) / "monitor_logs" / getTimestamp();
    fs::create_directories(directoryPath);
    logDir = directoryPath.string();



    monitorSub = nh.subscribe("/client_info", 1000, monitorDataCb);
    ros::spin();
    return 0;
}
