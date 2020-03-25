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

    XmlRpc::XmlRpcValue activeClients;
    nh.getParam("/activeClients", activeClients);

    if (activeClients.getType() != XmlRpc::XmlRpcValue::TypeArray || activeClients.size() == 0) {
        throw std::runtime_error("No clients defined in clientNamespaces param array");
    }

    for (int i = 0; i < activeClients.size(); ++i) {
        if (activeClients[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            throw std::runtime_error("Wrong element type of clientNamespaces param");
        }

        Client client(activeClients[i]);
        clients.insert(std::make_pair(activeClients[i], client));
    }


    int r1Start, r1End;
    nh.getParam("r1Start", r1Start);
    nh.getParam("r1End", r1End);

    Task t1;
    t1.command = Task::Commnands::PLAN_PATH;
    t1.path.first.point.uuid = r1Start;
    t1.path.second.point.uuid = r1End;
    Task t2;
    t2.command = Task::Commnands::PLAN_PATH;
    t2.path.first.point.uuid = r1End;
    t2.path.second.point.uuid = r1Start;
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);
    clients.at("r1").tasks.push(t1);
    clients.at("r1").tasks.push(t2);

    t1.path.first.point.uuid = 54;
    clients.at("r2").tasks.push(t1);
    clients.at("r2").tasks.push(t2);

    ROS_INFO("Server task manager launched successful");
}

bool TaskManagerServer::getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res) {

    RobotClients::iterator it = clients.find(req.clientId);
    if (it == clients.end()) {
        res.error.code = amr_msgs::GetTaskErrorCodes::UNKNOWN_CLIENT_ID;
        ROS_ERROR("Unknown client ID: %s", req.clientId.c_str());
        return true;
    }

    // check if some task exist
    if (it->second.tasks.empty()) {
        res.task.timeout = 5;
        res.task.taskId.id = amr_msgs::TaskId::DO_NOTHING;
        return true;
    }

    auto task = it->second.tasks.front();

    switch (task.command) {
        case Task::Commnands::PLAN_PATH: {
            amr_msgs::PlanPathNodes srv;
            srv.request.startUuid = task.path.first.point.uuid;
            srv.request.endUuid = task.path.second.point.uuid;

            if (planPathSrvClient.call(srv)) {
                if (srv.response.pathWaypoints.empty()) {
                    res.error.code = amr_msgs::GetTaskErrorCodes::PLANNING_FAILED;
                    return true;
                }

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

    it->second.tasks.pop();

    return true;
}

