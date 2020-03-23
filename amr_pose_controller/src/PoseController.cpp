//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/PoseController.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <future>


PoseController::PoseController(ros::NodeHandle& nh)
    :   performGoalAs(nh, "perform_goals", false) {

    ControllerConfig config;
    nh.getParam("controller/linearKp", config.linearKp);
    nh.getParam("controller/rotationKp", config.rotationKp);
    nh.getParam("controller/controllerFrequency", config.controllerFrequency);
    nh.getParam("controller/maxLinearSpeed", config.maxLinearSpeed);
    nh.getParam("controller/maxAngularSpeed", config.maxAngularSpeed);
    nh.getParam("controller/maxLinearAcceleration", config.maxLinearAcceleration);
    nh.getParam("controller/maxAngularAcceleration", config.maxAngularAcceleration);
    nh.getParam("controller/goalDeadZone", config.goalDeadZone);
    controller.reset(new Controller(config));

    nh.getParam("tf_prefix", tfPrefix);
    std::string cmdVelTopic;
    nh.getParam("cmdVelTopic", cmdVelTopic);
//    std::string robotPoseTopic;
//    nh.getParam("robotPoseTopic", robotPoseTopic);

    nh.getParam("waypointZone", waypointZone);

//    usleep(1000*1000*15);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(ros::this_node::getNamespace() + cmdVelTopic, 10);
    currentGoalPub = nh.advertise<amr_msgs::Point>("current_goal", 10);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &PoseController::robotPoseCb, this);
//    robotPoseSub = nh.subscribe(robotPoseTopic, 10, &PoseController::turtlesimPoseCb, this);


    performGoalAs.registerGoalCallback(boost::bind(&PoseController::acGoalCb, this));
    performGoalAs.start();

    // init semaphore client
    std::string semaphoreService;
    nh.getParam("semaphoreService", semaphoreService);
    semaphoreClient = std::make_shared<SemaphoreClient>(semaphoreService);
    semaphoreClient->unlockAllNodes();

    visual_tools.reset(new rvt::RvizVisualTools("map", "amr_current_goal"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();


    ros::Rate rate(config.controllerFrequency);
    while(ros::ok()) {
        ros::spinOnce();
        updateRobotPose();

        if (robotPoseReceived) {
            switch (state) {
                case State::INIT_STATE: {
                    // publish 0 velocity
                    cmdVelPub.publish(controller->getStopAction());

                    // if a waypoint is presented, lock it
                    if (!nodeLocked.valid() && !waypoints.empty()) {
                        nodeLocked = semaphoreClient->lockNodeAsync(waypoints.front());
                    }

                    // response form lock received
                    if (nodeLocked.valid()
                        && nodeLocked.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                        // is node locked
                        if (nodeLocked.get()) {
                            // set default required goal
                            currentRequiredGoal.uuid = -1;
                            currentRequiredGoal.pose = controller->getCurrentPose();
                            controller->setRequiredPose(currentRequiredGoal.pose);
                            currentGoalPub.publish(currentRequiredGoal);
                            state = State::GET_NEW_GOAL;
                        }
                        // else: node lock will be performed again (nodeLocked will be no valid)
                    }
                    break;
                }
                case State::GET_NEW_GOAL:
                    cmdVelPub.publish(controller->getControllerAction());

                    // check if next node has been locked successfully
                    if (nodeLocked.valid()
                        && nodeLocked.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
                        //result form previous node lock received
                        if (!nodeLocked.get()) {
                            // node has not been locked successfully, try it again
                            nodeLocked = semaphoreClient->lockNodeAsync(waypoints.front()); //todo timer call
                            rate.sleep();
                            continue;
                        }
                    }

                    if (!waypoints.empty()) {
                        // set new required goal
                        currentRequiredGoal = waypoints.front();
                        controller->setRequiredPose(currentRequiredGoal.pose);
                        currentGoalPub.publish(currentRequiredGoal);
                        waypoints.pop();
                        // lock next node
                        if (!waypoints.empty()) {
                            nodeLocked = semaphoreClient->lockNodeAsync(waypoints.front());
                        }
                        state = State::PERFORMING_GOAL;
                    }
                    break;
                case State::PERFORMING_GOAL:
                    cmdVelPub.publish(controller->getControllerAction());
                    publishAsFeedback();

                    // detect last goal zone
                    if (waypoints.empty() && controller->isZoneAchieved(config.goalDeadZone)) {
                        ROS_INFO("Last goal zone achieved");
                        publishAsResult();
                        currentRequiredGoal.uuid = -1;
                        currentRequiredGoal.pose = controller->getCurrentPose();
                        controller->setRequiredPose(currentRequiredGoal.pose);
                        currentGoalPub.publish(currentRequiredGoal);
                        state = State::GET_NEW_GOAL;
                        break;
                    }

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
            // stop the robot
            cmdVelPub.publish(controller->getStopAction());
            publishAsFeedback();
            ROS_WARN("No robot position received longer that %f [s].", 1 / config.controllerFrequency);
        }

        visualizeAndPublishCurrentGoal();
        rate.sleep();
    }
}

void PoseController::updateRobotPose() {
    tf::StampedTransform transform;
    try{
        if (state == State::INIT_STATE) {
            poseTfListener.waitForTransform("/map", tf::resolve(tfPrefix, "base_link"), ros::Time::now(), ros::Duration(5.0));
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
//void PoseController::robotPoseCb(const geometry_msgs::PoseConstPtr& poseMsg) {
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
//void PoseController::turtlesimPoseCb(const turtlesim::PoseConstPtr& poseMsg) {
//    currentRobotPose.theta = poseMsg->theta;
//    currentRobotPose.x = poseMsg->x;
//    currentRobotPose.y = poseMsg->y;
//    robotPoseReceived = true;
//}

void PoseController::acGoalCb() {
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

void PoseController::publishAsFeedback() {
    PerformGoalAs::Feedback feedback;
    feedback.currentGoal = currentRequiredGoal;
    feedback.currentPose = controller->getCurrentPose();

    performGoalAs.publishFeedback(feedback);
}

void PoseController::publishAsResult() {
    PerformGoalAs::Result result;
    performGoalAs.setSucceeded(result);
}

void PoseController::visualizeAndPublishCurrentGoal() {
    visual_tools->deleteAllMarkers();

    auto currentPose = controller->getCurrentPose();
    geometry_msgs::Point p1, p2;
    p1.x = currentPose.x;
    p1.y = currentPose.y;
    p2.x = currentRequiredGoal.pose.x;
    p2.y = currentRequiredGoal.pose.y;
    visual_tools->publishArrow(p1, p2, rvt::colors::GREEN, rvt::scales::LARGE);
    visual_tools->trigger();
}
