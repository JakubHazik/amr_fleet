//
// Created by jakub on 8.3.2020.
//

#include <amr_task_manager/Client/TaskManagerClient.h>
#include <std_srvs/SetBool.h>

TaskManagerClient::TaskManagerClient(ros::NodeHandle& nh)
    :   nh(nh),
        performWaypointsAc("pose_controller/perform_goals", true) {

    std::string getTaskService;
    nh.getParam("getTaskService", getTaskService);
    nh.getParam("client_id", clientId);

    resetTaskSrvServer = nh.advertiseService("reset_task", &TaskManagerClient::resetTaskServiceCb, this);
    getTaskSrvClient = nh.serviceClient<amr_msgs::GetTask>(getTaskService);
    enablePoseControlClient = nh.serviceClient<std_srvs::SetBool>(ros::this_node::getNamespace() + "/pose_controller/enable_pose_control");
    currentTaskPub = nh.advertise<amr_msgs::Task>("current_task", 10, true);

    performWaypointsAc.waitForServer();

    // wait for action server
    if (!performWaypointsAc.waitForServer(ros::Duration(5))) {
        ROS_ERROR("No action server presented");
        ros::shutdown();
    }

    if (!getTaskSrvClient.waitForExistence(ros::Duration(5))) {
        ROS_ERROR("There is no GetTask service server");
        ros::shutdown();
    }

    ROS_INFO("Client task manager is prepared");

    getNewTask();

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        rate.sleep();
    }
}

bool TaskManagerClient::resetTaskServiceCb(amr_msgs::ResetTask::Request& req, amr_msgs::ResetTask::Response& res) {
    ROS_INFO("Reset task received");

    switch (currentTask.taskId.id) {
        case amr_msgs::TaskId::PERFORM_WAYPOINTS: {
            performWaypointsAc.cancelAllGoals();
            break;
        }
        case amr_msgs::TaskId::TELEOPERATION: {
            std_srvs::SetBool srv;
            srv.request.data = true;
            enablePoseControlClient.call(srv);
            //todo check success
            break;
        }
    }

    callGetNewTaskServiceAfterTime(0.01);   // asap, but not in this function scope, service have to be done
    res.success = true;
    return true;
}

bool TaskManagerClient::getNewTask() {
    amr_msgs::GetTask taskSrv;

    taskSrv.request.clientId = clientId;
    if (!getTaskSrvClient.call(taskSrv)) {
        ROS_ERROR("Service get_new_task is not available, yet");
        callGetNewTaskServiceAfterTime(5);
        return false;
    }
    currentTaskPub.publish(taskSrv.response.task);
    switch (taskSrv.response.error.code) {
        case amr_msgs::GetTaskErrorCodes::OK: {
            ROS_INFO("Received new task");
            performTask(taskSrv.response.task);
            break;
        }
        case amr_msgs::GetTaskErrorCodes::INTERNAL_SYSTEM_ERROR: {
            ROS_ERROR("Get task respond with error: %d", taskSrv.response.error.code);
            callGetNewTaskServiceAfterTime(5);
            break;
        }
        case amr_msgs::GetTaskErrorCodes::PLANNING_FAILED: {
            ROS_ERROR("Get task respond with error: %d", taskSrv.response.error.code);
            callGetNewTaskServiceAfterTime(5);
            break;
        }
        case amr_msgs::GetTaskErrorCodes::UNKNOWN_CLIENT_ID: {
            ROS_ERROR("Get task respond with error: %d", taskSrv.response.error.code);
            callGetNewTaskServiceAfterTime(5);
            break;
        }

        default:
            throw std::runtime_error("Unknown error code");
    }
    return false;
}

void TaskManagerClient::callGetNewTaskServiceAfterTime(double time) {
    getNewTaskTimer = nh.createTimer(ros::Duration(time), std::bind(&TaskManagerClient::getNewTask, this), true);
}

bool TaskManagerClient::performTask(amr_msgs::Task& task) {
    ROS_INFO("New task id: %d", task.taskId.id);

    switch (task.taskId.id) {
        case amr_msgs::TaskId::PERFORM_WAYPOINTS: {
            // create goal
            amr_msgs::PerformGoalsGoal goal;
            goal.waypoinst = task.waypoints;
            performWaypointsAc.sendGoal(
                    goal,
                    boost::bind(&TaskManagerClient::acWaypointsDoneCb, this, _1, _2),
                    boost::bind(&TaskManagerClient::activeCallback, this),
                    boost::bind(&TaskManagerClient::acWaypointsFeedbackCb, this, _1));
            break;
        }
        case amr_msgs::TaskId::TELEOPERATION: {
            std_srvs::SetBool srv;
            srv.request.data = false;
            enablePoseControlClient.call(srv);
            //todo check success
            break;
        }
        case amr_msgs::TaskId::WAIT_FOR_USER_ACK: {
            ROS_WARN("Task WAIT_FOR_USER_ACK received, but it is not implemented yet");
            break;
        }
        case amr_msgs::TaskId::CHARGE_BATTERY: {
            ROS_WARN("Task CHARGE_BATTERY received, but it is not implemented yet");
            break;
        }
        case amr_msgs::TaskId::DO_NOTHING: {
            callGetNewTaskServiceAfterTime(task.timeout);
            ROS_INFO("Wait %f seconds", task.timeout);
            break;
        }
        default:
            ROS_ERROR("Unknown error code");
            throw std::runtime_error("Unknown error code");
    }
    currentTask = task;
    return false;
}

void TaskManagerClient::acWaypointsDoneCb(const actionlib::SimpleClientGoalState& state,
                                          const amr_msgs::PerformGoalsResultConstPtr& result) {
    // todo mutex for action callbacks

    if (state.state_ == state.SUCCEEDED) {
        ROS_INFO("Waypoints performed successfully");
        getNewTask();
        return;
    } else if (state.state_ == state.ABORTED) {
        ROS_WARN("Waypoints perform goal is aborted");
        return;
    }

}

void TaskManagerClient::activeCallback() {
//    ROS_INFO("Active callback");

}

void TaskManagerClient::acWaypointsFeedbackCb(const amr_msgs::PerformGoalsFeedbackConstPtr& feedback) {
//    ROS_INFO("Feedback callback");
}



