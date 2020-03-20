//
// Created by jakub on 15.3.2020.
//


#include <ros/ros.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "semaphore_client");
    ros::NodeHandle nh("~");


    ros::spin();

    return 0;
}

