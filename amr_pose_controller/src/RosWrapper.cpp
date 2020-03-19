//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/RosWrapper.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

RosWrapper::RosWrapper(ros::NodeHandle& nh)
    :   performGoalAs(nh, "perform_goals", false) {

    ControllerConfig config;
    nh.getParam("controller/linearKp", config.linearKp);
    nh.getParam("controller/rotationKp", config.rotationKp);
    nh.getParam("controller/controllerFrequency", config.controllerFrequency);
    nh.getParam("controller/maxLinearSpeed", config.maxLinearSpeed);
    nh.getParam("controller/maxLinearAcceleration", config.maxLinearAcceleration);
    controller.reset(new Controller(config));

    nh.getParam("tf_prefix", tfPrefix);
    std::string cmdVelTopic;
    nh.getParam("cmdVelTopic", cmdVelTopic);
//    std::string robotPoseTopic;
//    nh.getParam("robotPoseTopic", robotPoseTopic);

    nh.getParam("waypointZone", waypointZone);
    nh.getParam("goalZone", goalZone);

//    usleep(1000*1000*15);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(ros::this_node::getNamespace() + cmdVelTopic, 10);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::robotPoseCb, this);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &RosWrapper::turtlesimPoseCb, this);

    performGoalAs.registerGoalCallback(boost::bind(&RosWrapper::acGoalCb, this));
    performGoalAs.start();

    ros::Rate rate(config.controllerFrequency);
    while(ros::ok()) {
        ros::spinOnce();
        updateRobotPose();
        if (robotPoseReceived) {
            switch (state) {
                case State::GET_NEW_GOAL:
                    if (waypoints.empty()) {
                        // stop robot
                        cmdVelPub.publish(controller->getStopAction());
                    } else {
                        // set new required goal
                        currentRequiredGoal = waypoints.front();
                        controller->setRequiredPose(currentRequiredGoal.pose);
                        waypoints.pop();
                        state = State::PERFORMING_GOAL;
                    }
                    break;
                case State::PERFORMING_GOAL:
                    // detect last goal zone
                    if (waypoints.empty() && controller->isZoneAchieved(goalZone)) {
                        ROS_INFO("Last goal zone achieved");
                        state = State::GET_NEW_GOAL;
                        publishAsResult();
                        break;
                    }

                    cmdVelPub.publish(controller->getControllerAction());
                    publishAsFeedback();

                    // detect waypoint zone
                    if (!waypoints.empty() && controller->isZoneAchieved(waypointZone)) {
                        ROS_INFO("Waypoint zone achieved");
                        state = State::GET_NEW_GOAL;
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

void RosWrapper::updateRobotPose() {
    tf::StampedTransform transform;
    try{
        if (!robotPoseReceived) {
            poseTfListener.waitForTransform("/map", tf::resolve(tfPrefix, "base_link"), ros::Time::now(), ros::Duration(3.0));
        }

        poseTfListener.lookupTransform("/map", tf::resolve(tfPrefix, "base_link"), ros::Time(0), transform);
        geometry_msgs::Pose2D currentRobotPose;
        currentRobotPose.x = transform.getOrigin().x();
        currentRobotPose.y = transform.getOrigin().y();
        currentRobotPose.theta = tf::getYaw(transform.getRotation());

        controller->setCurrentPose(currentRobotPose);
        robotPoseReceived = true;
//        ROS_INFO_STREAM("X: " << currentRobotPose.x << " Y: " << currentRobotPose.y << " Yaw: " << currentRobotPose.theta);
    } catch (const tf::TransformException& ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

//
//void RosWrapper::robotPoseCb(const geometry_msgs::PoseConstPtr& poseMsg) {
//    double r, p, y;
//    tf::Quaternion q(
//            poseMsg->orientation.x,
//            poseMsg->orientation.y,
//            poseMsg->orientation.z,
//            poseMsg->orientation.w);
//    tf::Matrix3x3(q).getRPY(r, p, y);
//
//    currentRobotPose.x = poseMsg->position.x;
//    currentRobotPose.y = poseMsg->position.y;
//    currentRobotPose.theta = y;
//    robotPoseReceived = true;
//}
//
//void RosWrapper::turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg) {
//    currentRobotPose.theta = poseMsg->theta;
//    currentRobotPose.x = poseMsg->x;
//    currentRobotPose.y = poseMsg->y;
//    robotPoseReceived = true;
//}

void RosWrapper::acGoalCb() {
    ROS_INFO("New pose control goal received");
    auto goal = performGoalAs.acceptNewGoal();

    // clear waypoints queuq
    std::queue<amr_msgs::Point> empty;
    waypoints.swap(empty);

    // fill in queue
    for (const auto& point: goal->waypoinst) {
        waypoints.push(point);
    }

    state = State::GET_NEW_GOAL;
}

void RosWrapper::publishAsFeedback() {
    PerformGoalAs::Feedback feedback;
    feedback.currentGoal = currentRequiredGoal;
    feedback.currentPose = controller->getCurrentPose();

    performGoalAs.publishFeedback(feedback);
}

void RosWrapper::publishAsResult() {
    PerformGoalAs::Result result;
    performGoalAs.setSucceeded(result);
}
