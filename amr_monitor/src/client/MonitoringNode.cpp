//
// Created by jakub on 21. 3. 2020.
//

#include <ros/ros.h>
#include <amr_monitor/client/DataCollector.h>

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "monitor_client" );
    ros::NodeHandle nh("~");

    DataCollector dataCollector;

    ros::spin();
    return 0;
}