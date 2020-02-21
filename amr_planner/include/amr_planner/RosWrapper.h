//
// Created by jakub on 21.2.2020.
//

#ifndef PROJECT_ROSWRAPPER_H
#define PROJECT_ROSWRAPPER_H

#include <ros/ros.h>
#include <tuw_multi_robot_msgs/Graph.h>

#include <amr_planner/Graph.h>

class RosWrapper {
public:

    explicit RosWrapper(ros::NodeHandle& nh);

private:
    ros::Subscriber segSubscriber;
    Graph graph;

    void newSegmentsCb(const tuw_multi_robot_msgs::Graph::ConstPtr &segments);
};


#endif //PROJECT_ROSWRAPPER_H
