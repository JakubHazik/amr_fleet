//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/RosWrapper.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

RosWrapper::RosWrapper(ros::NodeHandle& nh)
    :   performGoalAs(nh, "perform_goals", false) {

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


    // setup AS

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(cmdVelTopic, 10);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::robotPoseCb, this);
    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::turtlesimPoseCb, this);

    performGoalAs.registerGoalCallback(boost::bind(&RosWrapper::acGoalCb, this));
    performGoalAs.start();



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
                        currentRequiredGoal = waypoints.front();
                        waypoints.pop();
                        state = State::PERFORMING_GOAL;
                    } else {
//                        ROS_INFO("STOP action");
                        geometry_msgs::Twist controllerAction = controller->getStopAction();
                        cmdVelPub.publish(controllerAction);
                    }
                    break;
                case State::PERFORMING_GOAL:
                    // detect last goal zone
                    if (waypoints.empty() && goalZone > controller->getDistanceError(currentRobotPose, currentRequiredGoal.pose)) {
                        ROS_INFO("Last goal zone achieved");
                        state = State::WAIT_FOR_GOAL;
                        publishAsResult();
                        break;
                    }

                    geometry_msgs::Twist controllerAction = controller->getControllerAction(currentRobotPose, currentRequiredGoal.pose);
                    cmdVelPub.publish(controllerAction);
                    publishAsFeedback();

                    // detect waypoint zone
                    if (!waypoints.empty() && waypointZone > controller->getDistanceError(currentRobotPose, currentRequiredGoal.pose)) {
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
    double r, p, y;
    tf::Quaternion q(
            poseMsg->orientation.x,
            poseMsg->orientation.y,
            poseMsg->orientation.z,
            poseMsg->orientation.w);
    tf::Matrix3x3(q).getRPY(r, p, y);

    currentRobotPose.x = poseMsg->position.x;
    currentRobotPose.y = poseMsg->position.y;
    currentRobotPose.theta = y;
    robotPoseReceived = true;
}

void RosWrapper::turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg) {
    currentRobotPose.theta = poseMsg->theta;
    currentRobotPose.x = poseMsg->x;
    currentRobotPose.y = poseMsg->y;
    robotPoseReceived = true;
}


void RosWrapper::acGoalCb() {
    auto goal = performGoalAs.acceptNewGoal();

    // clear waypoints queuq
    std::queue<amr_msgs::Point> empty;
    waypoints.swap(empty);

    // fill in queue
    for (const auto& point: goal->waypoinst) {
        waypoints.push(point);
    }

    state = State::WAIT_FOR_GOAL;
}


void RosWrapper::publishAsFeedback() {
    PerformGoalAs::Feedback feedback;
    feedback.currentGoal = currentRequiredGoal;
    feedback.currentPose = currentRobotPose;

    performGoalAs.publishFeedback(feedback);
}

void RosWrapper::publishAsResult() {
    PerformGoalAs::Result result;
    performGoalAs.setSucceeded(result);
}
