//
// Created by jakub on 8.3.2020.
//

#include <amr_task_manager/Server/TaskManagerServer.h>
#include <amr_msgs/GetTaskErrorCodes.h>
#include <amr_msgs/PlanPath.h>


//TODO tasky presunu budu predstavovati iba goaly, start bude zohladneny tak ze budeme vediet kde sa robot momentalne nahcadza

TaskManagerServer::TaskManagerServer(ros::NodeHandle& nh) {

    //todo configure
    parseTasks("/home/jakub/amr_ws/src/amr_fleet/amr_task_manager/config/tasks_config.yaml");

    std::string planPathService;
    nh.getParam("planPathService", planPathService);

    clientInfoSub = nh.subscribe("/client_info", 5, &TaskManagerServer::clientInfoCb, this);
    getTaskSrvServer = nh.advertiseService("get_task", &TaskManagerServer::getTaskCb, this);
    doCustomTaskServer = nh.advertiseService("do_custom_task", &TaskManagerServer::doCustomTaskAsapCb, this);
    planPathSrvClient = nh.serviceClient<amr_msgs::PlanPath>(planPathService);

    ROS_INFO("Server task manager launched successful");

//    ros::AsyncSpinner()

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

}

bool TaskManagerServer::getTaskCb(amr_msgs::GetTask::Request& req, amr_msgs::GetTask::Response& res) {
    auto client = getClient(req.clientId);

    // check if some task exist
    if (!client->isNewTaskAvailable()) {
        res.task.timeout = 5;
        res.task.taskId.id = amr_msgs::TaskId::DO_NOTHING;
        return true;
    }

    if (!client->isClientReady()) {
        // send empty task
        ROS_WARN("No client info received for client: %s yet, send empty task.", req.clientId.c_str());
        res.task.timeout = 1;
        res.task.taskId.id = amr_msgs::TaskId::DO_NOTHING;
        return true;
    }


    auto task = client->getNewTask();

    switch (task.taskId.id) {
        case amr_msgs::TaskId::PERFORM_WAYPOINTS: {
            amr_msgs::PlanPath srv;
            amr_msgs::Point startPoint;
            client->getCurrentPose(startPoint);
            startPoint.uuid = 0;
            srv.request.startPoint = startPoint;
            srv.request.endPoint.uuid = task.waypoints[0].uuid;

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
        case amr_msgs::TaskId::TELEOPERATION:
            res.task = task;
            res.error.code = amr_msgs::GetTaskErrorCodes::OK;
            break;
        case amr_msgs::TaskId::WAIT_FOR_USER_ACK:
            ROS_ERROR("WAIT_FOR_USER_ACK task is not implemented yet");
            break;
        case amr_msgs::TaskId::CHARGE_BATTERY:
            ROS_ERROR("CHARGE_BATTERY task is not implemented yet");
            break;
        case amr_msgs::TaskId::DO_NOTHING:
            res.task.timeout = task.timeout;
            res.task.taskId.id = task.taskId.id;
            return true;
    }
    return true;
}

bool TaskManagerServer::doCustomTaskAsapCb(amr_msgs::DoCustomTaskAsap::Request &req,
                                           amr_msgs::DoCustomTaskAsap::Response &res) {
    if (!req.cancelCustomTask) {
        // add custom task
        auto client = getClient(req.clientId);
        client->doCustomTaskAsap(req.task, req.resumePreviousTask);
        res.success = true;
    } else {
        // abort custom task which is now done
        auto client = getClient(req.clientId);
        client->resetCurrentTask();
        res.success = true;
    }
    return true;
}

std::shared_ptr<Client> TaskManagerServer::getClient(const std::string &clientId) {
    RobotClients::iterator it = clients.find(clientId);
    if (it == clients.end()) {
        ROS_ERROR("Unknown client ID: %s", clientId.c_str());
        throw std::out_of_range("Unknown client ID: " + clientId);
    }
    return it->second;
}

void TaskManagerServer::clientInfoCb(const amr_msgs::ClientInfoConstPtr& msg) {
    auto client = getClient(msg->clientId);
    client->setClientInfo(*msg);
}

void TaskManagerServer::parseTasks(const std::string &configFile) {
    YAML::Node config = YAML::LoadFile(configFile);

    YAML::Node clientsSeq = config["clients"];
    for (YAML::iterator it = clientsSeq.begin(); it != clientsSeq.end(); ++it) {
//        YAML::Node client = *it;
//        std::cout << client << std::endl << "======client" << std::endl;

        for (const auto &clientData : *it) {
//            std::cout << kv.first.as<std::string>() << "\n";    // r1
            std::string clientId = clientData.first.as<std::string>();
            YAML::Node tasksSeq = clientData.second;  // tasks seq

            auto client = std::make_shared<Client>(clientId);
            clients.insert(std::make_pair(clientId, client));

            for (YAML::iterator taskIt = tasksSeq.begin(); taskIt != tasksSeq.end(); ++taskIt) {
                auto task = parseTask(*taskIt);
                client->addNewTask(task);
            }
        }
    }
}

amr_msgs::Task TaskManagerServer::parseTask(const YAML::Node &taskNode) {
    int taskId = taskNode["task"]["taskId"].as<int>();
//    std::cout<< taskId<<std::endl;

    switch (taskId) {
        case 1: {
            amr_msgs::Task task;
            task.taskId.id = taskId;
            amr_msgs::Point point;
            point.uuid = taskNode["task"]["goalId"].as<int>();
            task.waypoints.push_back(point);
            return task;
        }
        case 4: {
            amr_msgs::Task task;
            task.taskId.id = taskId;
            task.timeout = taskNode["task"]["timeout"].as<int>();
            return task;
        }
        default: throw std::runtime_error("Unknown task ID, please check your task configuration file");
    }

}
