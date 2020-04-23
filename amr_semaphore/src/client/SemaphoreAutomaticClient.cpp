//
// Created by jakub on 21. 4. 2020.
//

#include <amr_semaphore/client/SemaphoreAutomaticClient.h>

SemaphoreAutomaticClient::SemaphoreAutomaticClient(const std::string &lockServiceName,
                                                   const std::string& setupServiceName,
                                                   int numLockedNodesBack,
                                                   int numLockedNodesAhead)
        : SemaphoreClient(lockServiceName, setupServiceName, numLockedNodesBack + numLockedNodesAhead),
          numLockedNodesBack(numLockedNodesBack),
          numLockedNodesAhead(numLockedNodesAhead) {

    unlockAllNodes();
    lockingThread = std::thread(&SemaphoreAutomaticClient::lockingLoop, this);
}

SemaphoreAutomaticClient::~SemaphoreAutomaticClient() {
    threadExitSignal.set_value();
    sleepingCv.notify_all();
}

void SemaphoreAutomaticClient::setNodeAsCurrentGoal(const amr_msgs::Point &node) {
    if (robotCurrentPointIt->uuid == node.uuid) {
        // do nothing
        sleepingCv.notify_all();
        return;
    }

    robotCurrentPointIt++;
    if (robotCurrentPointIt->uuid != node.uuid) {
        throw std::runtime_error("New current goal is not next node.");
    }

    sleepingCv.notify_all();
}

void SemaphoreAutomaticClient::setNewPath(const std::vector<amr_msgs::Point> &pathWaypoints) {
    unlockAllNodes();
    this->waypoints = pathWaypoints;
    robotCurrentPointIt = this->waypoints.begin();
    lockedNodesBack = this->waypoints.begin();
    lockedNodesFront = this->waypoints.begin();
    sleepingCv.notify_all();
}

bool SemaphoreAutomaticClient::isNodeLocked(const amr_msgs::Point &node) {
    for (auto lockedNodesIt = lockedNodesBack; lockedNodesIt != lockedNodesFront; lockedNodesIt++) {
        if (lockedNodesIt->uuid == node.uuid) {
            return true;
        }
    }

    return false;
}

void SemaphoreAutomaticClient::lockingLoop() {
    threadExitFut = threadExitSignal.get_future();

    ros::Rate rate(ros::Duration(0.2));
    while (threadExitFut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {

        if (std::distance(lockedNodesBack, robotCurrentPointIt) >= numLockedNodesBack) {
            if (unlockNode(*lockedNodesBack)) {
                // successfully unlocked
                lockedNodesBack++;
            } else {
                // unable to unlock next node, wait a moment and try it again
                rate.sleep();
            }
        } else if (std::distance(robotCurrentPointIt, lockedNodesFront) <= numLockedNodesAhead
            && lockedNodesFront != waypoints.end()) {
            if (lockNode(*lockedNodesFront)) {
                // successfully locked
                lockedNodesFront++;
                // accept automatic unlocking
//                if (std::distance(lockedNodesBack, lockedNodesFront) > numLockedNodesBack + numLockedNodesAhead) {
//                    lockedNodesBack++;
//                }
            } else {
                // unable to lock next node, wait a moment and try it again
                rate.sleep();
            }
        } else {
            std::unique_lock<std::mutex> lk(sleepingMutex);
            sleepingCv.wait(lk);   // this will wait until mutex will be unlocked
        }
    }
}

