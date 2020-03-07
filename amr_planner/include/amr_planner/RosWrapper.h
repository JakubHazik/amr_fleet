//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <amr_msgs/Graph.h>
#include <amr_msgs/PlanPathPoints.h>
#include <amr_msgs/PlanPathNodes.h>

#include <amr_planner/GraphSearchInterface.h>
#include <amr_planner/DataTypesAndConversions.h>
#include <amr_planner/Graph.h>


class RosWrapper {
public:

    explicit RosWrapper(ros::NodeHandle& nh);

//    std::vector<Node> getPlan(const Node& start, const Node& end);

private:
    ros::Subscriber segSubscriber;
    ros::ServiceServer planPathByPointsSrv;
    ros::ServiceServer planPathByNodesSrv;

    Graph graph;
    std::shared_ptr<GraphSearchInterface> graphSearch;

    void newGraphCb(const amr_msgs::Graph::ConstPtr& graphMsg);

    bool planPathPointsCallback(amr_msgs::PlanPathPoints::Request& req, amr_msgs::PlanPathPoints::Response& res);

    bool planPathNodesCallback(amr_msgs::PlanPathNodes::Request& req, amr_msgs::PlanPathNodes::Response& res);

};


#endif //PROJECT_ROSWRAPPER_H
