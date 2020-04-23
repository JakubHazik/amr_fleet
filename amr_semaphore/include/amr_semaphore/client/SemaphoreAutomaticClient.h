//
// Created by jakub on 21. 4. 2020.
//

#ifndef SRC_SEMAPHOREAUTOMATICCLIENT_H
#define SRC_SEMAPHOREAUTOMATICCLIENT_H

#include "SemaphoreClient.h"

class SemaphoreAutomaticClient : public SemaphoreClient {
public:
    SemaphoreAutomaticClient(const std::string& lockServiceName, const std::string& setupServiceName,
                                int numLockedNodesBack, int numLockedNodesAhead);

    ~SemaphoreAutomaticClient();

    void setNewPath(const std::vector<amr_msgs::Point>& pathWaypoints);

    void setNodeAsCurrentGoal(const amr_msgs::Point &node);

    bool isNodeLocked(const amr_msgs::Point &node);

private:

    std::vector<amr_msgs::Point> waypoints;
    std::vector<amr_msgs::Point> lockedNodes;
    std::vector<amr_msgs::Point>::iterator robotCurrentPointIt;
    std::vector<amr_msgs::Point>::iterator lockedNodesBack;
    std::vector<amr_msgs::Point>::iterator lockedNodesFront;

    int numLockedNodesAhead;
    int numLockedNodesBack;

    std::thread lockingThread;
    std::promise<void> threadExitSignal;
    std::future<void> threadExitFut;
    void lockingLoop();
    std::mutex sleepingMutex;
    std::condition_variable sleepingCv;

};


#endif //SRC_SEMAPHOREAUTOMATICCLIENT_H
