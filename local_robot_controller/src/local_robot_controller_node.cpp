//
// Created by jakub on 17.10.2019.
//

#include <ros/ros.h>
#include <local_robot_controller/local_robot_controller.h>


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "local_robot_controller_node" ); /// initializes the ros node with default name
    ros::NodeHandle nh;
    
    LocalRobotController localRobotController(nh);

//    ros::Subscriber sub = nh.subscribe("chatter", 1000, &LocalRobotController::callback, &localRobotController);





    return 0;
}