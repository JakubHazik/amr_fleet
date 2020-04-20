//
// Created by jakub on 17. 4. 2020.
//

#ifndef SRC_LOCKCONTAINERS_H
#define SRC_LOCKCONTAINERS_H


#include <queue>
#include <deque>
#include <set>
#include <list>
#include <memory>

#include <amr_graph_representation/Graph.h>
#include <amr_graph_representation/DataTypesAndConversions.h>

class AreaBasedLocks;

class NodesOccupancyContainer {
public:
    NodesOccupancyContainer();

    void setupClient(const std::string &ownerId, unsigned int occupancyLength);

    bool lockNode(const std::string &ownerId, const Node &node, bool lockVirtually = false);

    bool unlockNode(const std::string& ownerId, const Node& node);

    void unlockAllNodes(const std::string& ownerId);

    std::map<std::string, std::list<Node>> getOccupancyData();

    bool isNodeAlreadyLocked(const Node& node);

    bool isNodeAlreadyLockedBy(const std::string& ownerId, const Node& node);

    // check nodes which are out out of mx buffer range and remove it
    // function return vector of removed nodes
    std::vector<Node> checkMaxNodesAndRemove(const std::string& ownerId, const Node& referencedNode);

    // return owner ID if node is locked in critical buffer of robot length
    std::string isNodeReallyLocked(const Node& node);

    // return owner ID if node is locked and is not in critical buffer of robot length (bidirectioanl nodes)
    std::string isNodeVirtuallyLocked(const Node& node);

private:
    std::map<std::string, std::list<Node>> data;
    std::map<std::string, unsigned int> clientOccupancyLength;
};



class AreaBasedLocks {
public:
    AreaBasedLocks() = default;

    void setupArea(std::set<int> areaNodeUuids, unsigned int maxClientsInArea);

    bool lockNode(const std::string& ownerId, const Node& node);

    void unlockNode(const std::string& ownerId, const Node& node);

    void unlockNode(const std::string& ownerId, const std::vector<Node>& nodes);

    bool canBeNodeLocked(const std::string& ownerId, const Node& node);

    void unlockAllNodes(const std::string& ownerId);

private:
    typedef struct {
        std::set<int> areaNodes;
        std::map<std::string, std::list<Node>> clientsLockedNodes;
        unsigned int maxClientsNum;     // max number of clients in area
    } Area;

    std::vector<Area> data;
};



#endif //SRC_LOCKCONTAINERS_H
