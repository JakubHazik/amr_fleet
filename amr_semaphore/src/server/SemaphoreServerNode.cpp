//
// Created by jakub on 15.3.2020.
//


#include <ros/ros.h>
#include <amr_semaphore/server/SemaphoreServer.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "semaphore_server");
    ros::NodeHandle nh("~");

    SemaphoreServer semaphoreServer(nh);

    ros::spin();

    return 0;
}

