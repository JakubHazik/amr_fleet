//
// Created by jakub on 24. 3. 2020.
//

#include <amr_gui/control/ControlInterface.h>

#include <geometry_msgs/Twist.h>


ControlInterface::ControlInterface() {
    // todo configure service names

    setNodePropertiesSrv = nh.serviceClient<amr_msgs::SetNodeProperties>("/planner/set_node_properties");
    doCustomTaskSrv = nh.serviceClient<amr_msgs::DoCustomTaskAsap>("/task_manager_server/do_custom_task");
    semaphoreClient = std::make_shared<SemaphoreClient>("/semaphore_server/lock_node", "/semaphore_server/setup_semaphore");
}

Result ControlInterface::setNodeReachability(unsigned int nodeUuid, bool reachable) {
    // set node as unreachable
    amr_msgs::SetNodeProperties srv;
    srv.request.reachability = reachable;
    srv.request.point.uuid = nodeUuid;
    setNodePropertiesSrv.call(srv);

    amr_msgs::Point point;
    point.uuid = nodeUuid;
    if (reachable) {
        semaphoreClient->unlockNode(point);
    } else {
        semaphoreClient->lockNode(point);
    }

    return Result(srv.response.success, srv.response.message);
}

Result ControlInterface::startTeleopClient(const std::string &clientId) {
    amr_msgs::DoCustomTaskAsap srv;
    srv.request.clientId = clientId;
    srv.request.task.taskId.id = amr_msgs::TaskId::TELEOPERATION;
    srv.request.resumePreviousTask = true;
    doCustomTaskSrv.call(srv);
    if (!srv.response.success) {
        return {srv.response.success, srv.response.message};
    }

    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/" + clientId + "/mobile_base/commands/velocity", 10);
    velLinear = 0;
    velRot = 0;
    cmdVelTimer = nh.createTimer(ros::Duration(0.1), std::bind(&ControlInterface::publishTeleopSpeed, this));
    return {srv.response.success, srv.response.message};
}

Result ControlInterface::stopTeleopClient(const std::string &clientId) {
    amr_msgs::DoCustomTaskAsap srv;
    srv.request.clientId = clientId;
    srv.request.resumePreviousTask = true;
    srv.request.cancelCustomTask = true;
    doCustomTaskSrv.call(srv);
    cmdVelTimer.stop();
    cmdVelPub.shutdown();
    return {srv.response.success, srv.response.message};
}

void ControlInterface::setTeleopSpeed(double linear, double rotation) {
    velLinear = linear / 3;
    velRot = rotation / 3;
}

void ControlInterface::publishTeleopSpeed() {
    geometry_msgs::Twist msg;
    msg.linear.x = velLinear;
    msg.angular.z = velRot;
    cmdVelPub.publish(msg);
}
