//
// Created by jakub on 7.3.2020.
//

#include <ros/ros.h>
#include <amr_pose_controller/Controller.h>
#include <amr_pose_controller/RosWrapper.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "pose_controller");
    ros::NodeHandle nh("~");

    RosWrapper robotController(nh);

    return 0;
}