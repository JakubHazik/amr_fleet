//
// Created by jakub on 17.10.2019.
//

#include <local_robot_controller/local_robot_controller.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Trigger.h>

// todo service for reset path/points queue
//todo improve statuses checking
// todo escalating to SIGTERM

LocalRobotController::LocalRobotController(ros::NodeHandle &nh): nh(nh), mbClient("move_base", true) {

    pathSubscriber = nh.subscribe("path_synced", 100, &LocalRobotController::subPathCb, this);
    amclPoseSubscriber = nh.subscribe("amcl_pose", 2, &LocalRobotController::subAmclPoseCb, this);
    resetGoalsSrvServer = nh.advertiseService("reset_goals", &LocalRobotController::resetGoalsServiceCb, this);
    
    nh.param<double>("waypoint_zone", waypointZone, 0.20);


    ros::Rate r ( 10 );

    while ( ros::ok() ) {
        r.sleep();
        ros::spinOnce();
        updatePose();
    }
}

void LocalRobotController::updatePose() {
    // this function is called periodically

    while(!mbClient.waitForServer(ros::Duration(5.0))){
        ROS_ERROR("Waiting for the move_base action server to come up");
    }

    if (goalIsProcessing) {
        if (requiredPoses.size() > 1 && waypointZone > calculatePosesDistance(requiredPoses.front(), robotActualPose.pose)) {
            ROS_ERROR("Points distance: %f",  calculatePosesDistance(requiredPoses.front(), robotActualPose.pose));
            // robot achieve a near range around waypoint
            ROS_ERROR("Robot achieve a near range around waypoint");
            requiredPoses.erase(requiredPoses.begin());
            sendNewGoal();
            return;
        }

        if (mbClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            // goal has been successfully achieved
            ROS_ERROR("Goal has been successfully achieved");

            // remove achieved goal
            requiredPoses.erase(requiredPoses.begin());
            if (!requiredPoses.empty()) {
                sendNewGoal();
            } else {
                goalIsProcessing = false;
                return;
            }

        } else {
            // todo chcek error
            return;
        }
    } else {
        // no goal is in processing, check if we have a goal in stack

        if (!requiredPoses.empty()) {
            sendNewGoal();
        }
    }
}

void LocalRobotController::sendNewGoal() {
    // fill goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = requiredPoses.front();
    mbClient.sendGoal(goal);

    goalIsProcessing = true;

    ROS_ERROR("New goal has been sent");
}

void LocalRobotController::subAmclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
    robotActualPose = msg->pose;
}

void LocalRobotController::subPathCb(const nav_msgs::Path::ConstPtr& msg) {
    ROS_ERROR("New path sequence has been received");

    for (auto pose: msg->poses) {
        requiredPoses.emplace_back(pose.pose);
    }
}

double LocalRobotController::calculatePosesDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2) {
    return sqrt(pow(p1.position.x - p2.position.x, 2) +  pow(p1.position.y - p2.position.y, 2));
}

bool LocalRobotController::resetGoalsServiceCb(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    requiredPoses.clear();
    mbClient.cancelAllGoals();
    return true;
}

