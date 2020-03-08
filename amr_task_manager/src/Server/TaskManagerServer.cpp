//
// Created by jakub on 8.3.2020.
//

#include <amr_task_manager/Server/TaskManagerServer.h>
#include <amr_msgs/GetTaskErrorCodes.h>
#include <amr_msgs/PlanPathNodes.h>


TaskManagerServer::TaskManagerServer(ros::NodeHandle& nh) {

    std::string planPathByNodesService;
    nh.getParam("planPathByNodesService", planPathByNodesService);

    getTaskSrvServer = nh.advertiseService("get_task", &TaskManagerServer::getTaskCb, this);
    planPathSrvClient = nh.serviceClient<amr_msgs::PlanPathNodes>(planPathByNodesService);

}

bool TaskManagerServer::getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res) {

    RobotsTasks::iterator it = tasks.find(req.clientId);
    if (it == tasks.end()) {
        res.error.code = amr_msgs::GetTaskErrorCodes::UNKNOWN_CLIENT_ID;
        return true;
    }

    auto task = it->second.front();

    switch (task.command) {
        case Task::Commnands::PLAN_PATH: {
            amr_msgs::PlanPathNodes srv;
            srv.request.startUuid = task.path.first.point.uuid;
            srv.request.endUuid = task.path.second.point.uuid;

            if (planPathSrvClient.call(srv)) {
                res.task.waypoints = srv.response.pathWaypoints;
                res.task.taskId.id = amr_msgs::TaskId::PERFORM_WAYPOINTS;
                res.error.code = amr_msgs::GetTaskErrorCodes::OK;
            } else {
                ROS_ERROR("Failed to call service to plan path");
                res.error.code = amr_msgs::GetTaskErrorCodes::INTERNAL_SYSTEM_ERROR;
            }
            break;
        }
        case Task::Commnands::WAIT_FOR_USER_ACK:
            ROS_ERROR("WAIT_FOR_USER_ACK task is not implemented yet");
            break;
        case Task::Commnands::CHARGE_BATTERY:
            ROS_ERROR("CHARGE_BATTERY task is not implemented yet");
            break;
        case Task::Commnands::DO_NOTHING:
            ROS_ERROR("DO_NOTHING task is not implemented yet");
            break;
    }

    return true;
}

