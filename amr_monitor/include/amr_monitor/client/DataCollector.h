//
// Created by jakub on 21. 3. 2020.
//

#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <amr_msgs/Point.h>
#include <amr_msgs/ClientInfo.h>


class DataCollector {
public:
    DataCollector();

private:

    ros::Subscriber robotPoseSub;
    ros::Subscriber robotCurrentGoalSub;
    ros::Publisher clientStatusPub;

    amr_msgs::ClientInfo clientInfo;

    void robotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
    void robotCurrentGoalCb(const amr_msgs::PointConstPtr& point);

};


#endif //SRC_DATACOLLECTOR_H
