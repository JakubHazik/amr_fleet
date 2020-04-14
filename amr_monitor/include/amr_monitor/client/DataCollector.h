//
// Created by jakub on 21. 3. 2020.
//

#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <amr_msgs/Point.h>
#include <amr_msgs/ClientInfo.h>
#include <amr_msgs/Task.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


class DataCollector {
public:
    DataCollector();

private:

    ros::Subscriber robotPoseSub;
    ros::Subscriber robotCurrentGoalSub;
    ros::Subscriber currentTaskSub;
    ros::Publisher clientStatusPub;
    tf::TransformListener poseTfListener;
    std::string tfPrefix;

    amr_msgs::ClientInfo clientInfo;

    bool robotCurrentPoseReceived = false;

    bool readRobotPoseTf();
    void robotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
    void robotCurrentGoalCb(const amr_msgs::PointConstPtr& point);
    void currentTaskCb(const amr_msgs::TaskConstPtr& task);

};


#endif //SRC_DATACOLLECTOR_H
