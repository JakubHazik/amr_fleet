//
// Created by jakub on 24. 3. 2020.
//

#include <amr_gui/control/ControlInterface.h>


ControlInterface::ControlInterface() {
    ros::NodeHandle nh;

    // todo configure service names

    setNodePropertiesSrv = nh.serviceClient<amr_msgs::SetNodeProperties>("/planner/set_node_properties");

    semaphoreClient = std::make_shared<SemaphoreClient>("/semaphore_server/lock_node");
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
