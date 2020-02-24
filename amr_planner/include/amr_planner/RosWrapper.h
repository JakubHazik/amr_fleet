//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h>
#include <amr_planner/GraphSearchInterface.h>
#include <amr_planner/GraphSearchMultiRobot.h>
#include <std_srvs/Trigger.h>

#include <amr_planner/Graph.h>
#include <memory>
#include <vector>

class RosWrapper {
public:

    explicit RosWrapper(ros::NodeHandle& nh);

    std::vector<Node> getPlan(const Node& start, const Node& end);

private:
    ros::Subscriber segSubscriber;
    ros::ServiceServer planPathSrv;

    Graph graph;
    std::shared_ptr<GraphSearchMultiRobot> graphSearch;

    void newSegmentsCb(const tuw_multi_robot_msgs::Graph::ConstPtr &segments);

    bool planPathSrvCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

};


#endif //PROJECT_ROSWRAPPER_H
