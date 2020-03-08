//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/RosWrapper.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

RosWrapper::RosWrapper(ros::NodeHandle& nh) {

    ControllerConfig config;
    nh.getParam("controller/linearKp", config.linearKp);
    nh.getParam("controller/rotationKp", config.rotationKp);
    nh.getParam("controller/controllerFrequency", config.controllerFrequency);
    nh.getParam("controller/maxLinearSpeed", config.maxLinearSpeed);
    nh.getParam("controller/maxLinearAcceleration", config.maxLinearAcceleration);
    controller.reset(new Controller(config));

    std::string cmdVelTopic;
    nh.getParam("cmdVelTopic", cmdVelTopic);
    std::string robotPoseTopic;
    nh.getParam("robotPoseTopic", robotPoseTopic);

    nh.getParam("waypointZone", waypointZone);
    nh.getParam("goalZone", goalZone);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 10);
    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::robotPoseCb, this);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::turtlesimPoseCb, this);


//    geometry_msgs::Pose2D wp;
//    wp.x = 1;
//    wp.y = 1;
//    waypoints.push(wp);
//    wp.x = 1;
//    wp.y = 9;
//    waypoints.push(wp);
//    wp.x = 9;
//    wp.y = 9;
//    waypoints.push(wp);
//    wp.x = 1;
//    wp.y = 1;
//    waypoints.push(wp);


    ros::Rate rate(config.controllerFrequency);
    while(ros::ok()) {
        ros::spinOnce();

        if (robotPoseReceived) {
            switch (state) {
                case State::WAIT_FOR_GOAL:
                    if (!waypoints.empty()) {
                        ROS_INFO("Select first waypoint");
                        requiredGoal = waypoints.front();
                        waypoints.pop();
                        state = State::PERFORMING_GOAL;
                    } else {
                        ROS_INFO("STOP action");
                        geometry_msgs::Twist controllerAction = controller->getStopAction();
                        cmdVelPub.publish(controllerAction);
                    }
                    break;
                case State::PERFORMING_GOAL:
                    // detect last goal zone
                    if (waypoints.empty() && goalZone > controller->getDistanceError(robotCurrentPose, requiredGoal)) {
                        ROS_INFO("Last goal zone achieved");
                        state = State::WAIT_FOR_GOAL;
                        break;
                    }

                    geometry_msgs::Twist controllerAction = controller->getControllerAction(robotCurrentPose, requiredGoal);
                    cmdVelPub.publish(controllerAction);

                    // detect waypoint zone
                    if (!waypoints.empty() && waypointZone > controller->getDistanceError(robotCurrentPose, requiredGoal)) {
                        ROS_INFO("Waypoint zone achieved");
                        state = State::WAIT_FOR_GOAL;
                        break;
                    }

                    break;
            }

            robotPoseReceived = false;
        } else {
            ROS_WARN("No robot position received longer that %f [s].", 1 / config.controllerFrequency);
        }

        rate.sleep();
    }
}

void RosWrapper::robotPoseCb(const geometry_msgs::PoseConstPtr& poseMsg) {
//    ROS_INFO("Robot pose received");
    double r, p, y;
    tf::Quaternion q(
            poseMsg->orientation.x,
            poseMsg->orientation.y,
            poseMsg->orientation.z,
            poseMsg->orientation.w);
    tf::Matrix3x3(q).getRPY(r, p, y);

    robotCurrentPose.x = poseMsg->position.x;
    robotCurrentPose.y = poseMsg->position.y;
    robotCurrentPose.theta = y;
    robotPoseReceived = true;
}

void RosWrapper::turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg) {
//    ROS_INFO("Robot pose received");
    robotCurrentPose.theta = poseMsg->theta;
    robotCurrentPose.x = poseMsg->x;
    robotCurrentPose.y = poseMsg->y;
    robotPoseReceived = true;
}

void zoneAchieved() {

}

