//
// Created by jakub on 8.3.2020.
//

#include <amr_task_manager/Server/TaskManagerServer.h>
#include <amr_msgs/GetTaskErrorCodes.h>
#include <amr_msgs/PlanPathNodes.h>

//TODO tasky presunu budu predstavovati iba goaly, start bude zohladneny tak ze budeme vediet kde sa robot momentalne nahcadza

TaskManagerServer::TaskManagerServer(ros::NodeHandle& nh) {

    std::string planPathByNodesService;
    nh.getParam("planPathByNodesService", planPathByNodesService);

    getTaskSrvServer = nh.advertiseService("get_task", &TaskManagerServer::getTaskCb, this);
    pauseClientTask = nh.advertiseService("do_custom_task", &TaskManagerServer::doCustomTaskAsapCb, this);
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



    amr_msgs::Task t1;
    t1.taskId.id = amr_msgs::TaskId::PERFORM_WAYPOINTS;
    t1.waypoints.push_back({});
    t1.waypoints.push_back({});
    t1.waypoints[0].uuid = r1Start;
    t1.waypoints[1].uuid = r1End;
    amr_msgs::Task t2;
    t2.taskId.id = amr_msgs::TaskId::PERFORM_WAYPOINTS;
    t2.waypoints.push_back({});
    t2.waypoints.push_back({});
    t2.waypoints[0].uuid = r1End;
    t2.waypoints[1].uuid = r1Start;

    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);
    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);
    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);
    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);
    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);
    clients.at("r1").addNewTask(t1);
    clients.at("r1").addNewTask(t2);

    t1.waypoints[0].uuid = 54;
    clients.at("r2").addNewTask(t1);
    clients.at("r2").addNewTask(t2);




//
//    Task wait;
//    wait.command = Task::Commnands::DO_NOTHING;
//    wait.timeout = 20;
//
//    Task t1;
//    t1.command = Task::Commnands::PLAN_PATH;
//    Task t2;
//    t2.command = Task::Commnands::PLAN_PATH;
//
//    t1.path.first.point.uuid = 29;
//    t1.path.second.point.uuid = 63;
//    t2.path.first.point.uuid = 1;
//    t2.path.second.point.uuid = 63;
//
////    clients.at("r1").tasks.push(wait);
////    clients.at("r2").tasks.push(wait);
//
//    clients.at("r1").tasks.push(t1);
//    clients.at("r2").tasks.push(t2);

    ROS_INFO("Server task manager launched successful");
}

bool TaskManagerServer::getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res) {

    Client client;
    try {
        client = getClient(req.clientId);
    } catch (const std::out_of_range& ex) {
        res.error.code = amr_msgs::GetTaskErrorCodes::UNKNOWN_CLIENT_ID;
        return true;
    }

    // check if some task exist
    if (!client.isNewTaskAvailable()) {
        res.task.timeout = 5;
        res.task.taskId.id = amr_msgs::TaskId::DO_NOTHING;
        return true;
    }

    auto task = client.getNewTask();

    switch (task.taskId.id) {
        case amr_msgs::TaskId::PERFORM_WAYPOINTS: {
            amr_msgs::PlanPathNodes srv;
            srv.request.startUuid = task.waypoints[0].uuid;
            srv.request.endUuid = task.waypoints[1].uuid;

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
        case amr_msgs::TaskId::WAIT_FOR_USER_ACK:
            ROS_ERROR("WAIT_FOR_USER_ACK task is not implemented yet");
            break;
        case amr_msgs::TaskId::CHARGE_BATTERY:
            ROS_ERROR("CHARGE_BATTERY task is not implemented yet");
            break;
        case amr_msgs::TaskId::DO_NOTHING:
            ROS_ERROR("DO_NOTHING task is not implemented yet");
            break;
    }
    return true;
}

bool TaskManagerServer::doCustomTaskAsapCb(amr_msgs::DoCustomTaskAsap::Request &req,
                                           amr_msgs::DoCustomTaskAsap::Response &res) {
    if (!req.cancel) {
        auto client = getClient(req.clientId);
        client.doCustomTaskAsap(req.task, req.performPreviousTaskAgain);
    } else {
        auto client = getClient(req.clientId);
        client.resetCurrentTask();
    }
    return false;
}

Client TaskManagerServer::getClient(const std::string &clientId) {
    RobotClients::iterator it = clients.find(clientId);
    if (it == clients.end()) {
        ROS_ERROR("Unknown client ID: %s", clientId.c_str());
        throw std::out_of_range("Unknown client ID: " + clientId);
    }
    return it->second;
}

