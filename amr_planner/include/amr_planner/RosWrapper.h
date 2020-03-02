//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <ros/ros.h>
#include <amr_msgs/Graph.h>
#include <amr_planner/GraphSearchInterface.h>
#include <amr_planner/GraphSearchMultiRobot.h>
#include <std_srvs/Trigger.h>
#include <amr_planner/PlanPath.h>
#include <amr_planner/DataTypesAndConversions.h>

#include <amr_planner/Graph.h>
#include <memory>
#include <vector>

class RosWrapper {
public:

    explicit RosWrapper(ros::NodeHandle& nh);

//    std::vector<Node> getPlan(const Node& start, const Node& end);

private:
    ros::Subscriber segSubscriber;
    ros::ServiceServer planPathSrv;

    Graph graph;
    std::shared_ptr<GraphSearchMultiRobot> graphSearch;

    void newGraphCb(const amr_msgs::Graph::ConstPtr& graphMsg);

    bool planPathSrvCallback(amr_planner::PlanPathRequest& req, amr_planner::PlanPathResponse& res);

};


#endif //PROJECT_ROSWRAPPER_H
