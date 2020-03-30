//
// Created by jakub on 24. 3. 2020.
//

#ifndef SRC_CONTROLINTERFACE_H
#define SRC_CONTROLINTERFACE_H

#include <memory>

#include <ros/ros.h>
#include <amr_msgs/SetNodeProperties.h>
#include <amr_semaphore/client/SemaphoreClient.h>


typedef std::pair<bool, std::string> Result;


class ControlInterface {
public:
    ControlInterface();

    Result setNodeReachability(unsigned int nodeUuid, bool reachable);

private:
    ros::ServiceClient setNodePropertiesSrv;

    std::shared_ptr<SemaphoreClient> semaphoreClient;


};


#endif //SRC_CONTROLINTERFACE_H
