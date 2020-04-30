//
// Created by jakub on 23. 4. 2020.
//

#include <ros/ros.h>
#include <amr_msgs/ClientInfo.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace rvt = rviz_visual_tools;
rvt::RvizVisualToolsPtr visual_tools;

std::map<std::string, geometry_msgs::Pose> clientPoses;

void monitorDataCb(const amr_msgs::ClientInfoPtr& msg) {
    clientPoses[msg->clientId] = msg->poseWithCovariance.pose.pose;
}


void drawPoses() {
    visual_tools->deleteAllMarkers();
    for (auto& client: clientPoses) {
        visual_tools->publishArrow(client.second, rvt::colors::GREEN, rvt::scales::XLARGE, 0.15);
        geometry_msgs::Pose textPose = client.second;
        textPose.position.x += 0.1;
        visual_tools->publishText(textPose, client.first, rvt::colors::GREEN, rvt::scales::XXXLARGE, false);
        visual_tools->publishCylinder(client.second, rvt::colors::TRANSLUCENT_DARK, 0.01, 0.35);
    }
    visual_tools->trigger();
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "monitor_visualizer" );
    ros::NodeHandle nh;

    ros::Subscriber monitorSub = nh.subscribe("/client_info", 1000, monitorDataCb);

    visual_tools.reset(new rvt::RvizVisualTools("map", "/rviz_client_info"));
    visual_tools->loadMarkerPub(false, true);  // create publisher before waiting

    // Clear messages
    visual_tools->deleteAllMarkers();
    visual_tools->enableBatchPublishing();

    ros::Rate r(2);
    while (ros::ok()) {
        ros::spinOnce();
        drawPoses();
        r.sleep();
    }

    return 0;
}
