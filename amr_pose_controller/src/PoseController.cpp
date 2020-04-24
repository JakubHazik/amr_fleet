//
// Created by jakub on 7.3.2020.
//

#include <amr_pose_controller/PoseController.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <future>
#include <sstream>


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
    nh.getParam("waypointZone", waypointZone);
    poseControlSwitchSrv = nh.advertiseService("enable_pose_control", &PoseController::poseControlSwitchCb, this);

    cmdVelPub = nh.advertise<geometry_msgs::Twist>(ros::this_node::getNamespace() + cmdVelTopic, 10);
    currentGoalPub = nh.advertise<amr_msgs::Point>("current_goal", 10);

    performGoalAs.registerGoalCallback(boost::bind(&PoseController::acGoalCb, this));
    performGoalAs.registerPreemptCallback(boost::bind(&PoseController::acCancelCb, this));
    performGoalAs.start();

    // init semaphore client
    std::string semaphoreLockService;
    nh.getParam("semaphoreLockService", semaphoreLockService);
    std::string semaphoreSetupService;
    nh.getParam("semaphoreSetupService", semaphoreSetupService);
    std::map<std::string, int> semaphoreLockedNodesParam;
    nh.getParam("semaphoreLockedNodes", semaphoreLockedNodesParam);
    semaphoreClient = std::make_shared<SemaphoreAutomaticClient>(semaphoreLockService, semaphoreSetupService,
                         semaphoreLockedNodesParam["robotSize"], semaphoreLockedNodesParam["robotSpeedForward"]);

    visual_tools.reset(new rvt::RvizVisualTools("map", "amr_current_goal"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();


    usleep(1000 * 1000 * 1);
    ros::Rate rate(config.controllerFrequency);
    while(ros::ok()) {
        ros::spinOnce();
        updateRobotPose();

        if (robotPoseReceived) {
            switch (state) {
                case State::INIT_STATE: {
                    // publish 0 velocity
                    controller->setRequiredPose(controller->getCurrentPose());
                    cmdVelPub.publish(controller->getStopAction());
                    state = State::GET_NEW_GOAL;
                    break;
                }
                case State::GET_NEW_GOAL: {
                    std::lock_guard<std::mutex> lk(waypointsMutex);
                    cmdVelPub.publish(controller->getControllerAction());
                    if (!waypoints.empty() && semaphoreClient->isNodeLocked(waypoints.front())) {
                        // set new required goal
                        currentRequiredGoal = waypoints.front();
                        semaphoreClient->setNodeAsCurrentGoal(waypoints.front());
                        controller->setRequiredPose(currentRequiredGoal.pose);
                        currentGoalPub.publish(currentRequiredGoal);
                        waypoints.pop_front();
                        state = State::PERFORMING_GOAL;
                    }
                }
                break;
                case State::PERFORMING_GOAL: {
                    cmdVelPub.publish(controller->getControllerAction());
                    publishAsFeedback();

                    std::lock_guard<std::mutex> lk(waypointsMutex);
                    // detect last goal zone
                    if (waypoints.empty() && controller->isZoneAchieved(config.goalDeadZone)) {
//                        ROS_INFO("Last goal zone achieved");
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
//                        ROS_INFO("Waypoint zone achieved");
                        state = State::GET_NEW_GOAL;
                        break;
                    }
                }
                break;
                case State::NO_POSE_CONTROL:
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

void PoseController::acGoalCb() {
    ROS_INFO("New pose control goal received");
    auto goal = performGoalAs.acceptNewGoal();

    waypointsMutex.lock();
    // clear waypoints queue
    waypoints = {};
    // fill in queue
    for (const auto& point: goal->waypoinst) {
        waypoints.push_back(point);
    }
    waypointsMutex.unlock();

    printWaypoints();

    // call it async due to service calls inside
    std::async(std::launch::async, &SemaphoreAutomaticClient::setNewPath, semaphoreClient, goal->waypoinst);
}

void PoseController::acCancelCb() {
    ROS_INFO("Cancel goal");
    std::lock_guard<std::mutex> lk(waypointsMutex);
    waypoints = {};  //clear waypoints queue
    publishAsResult();
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

    if (state == State::PERFORMING_GOAL) {
        auto currentPose = controller->getCurrentPose();
        geometry_msgs::Point p1, p2;
        p1.x = currentPose.x;
        p1.y = currentPose.y;
        p2.x = currentRequiredGoal.pose.x;
        p2.y = currentRequiredGoal.pose.y;
        visual_tools->publishArrow(p1, p2, rvt::colors::GREEN, rvt::scales::LARGE);
        visual_tools->trigger();
    }
}

bool PoseController::poseControlSwitchCb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    if (req.data) {
        // enable pose control
        if (state != State::NO_POSE_CONTROL) {
            ROS_ERROR("Pose is already controlled.");
            res.success = false;
            return true;
        }
        state = State::GET_NEW_GOAL;
    } else {
        // disable pose control
        if (state == State::PERFORMING_GOAL) {
            ROS_ERROR("Unable to disable pose control now, cancel goal first.");
            res.success = false;
            return true;
        }
        state = State::NO_POSE_CONTROL;
    }
    res.success = true;
    return true;
}

void PoseController::printWaypoints() {
    std::lock_guard<std::mutex> lk(waypointsMutex);
    std::stringstream ss;
    for (auto& point: waypoints) {
        ss << point.uuid << " -> ";
    }

    ROS_INFO("Waypoints: %s", ss.str().c_str());
}