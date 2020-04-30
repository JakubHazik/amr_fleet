//
// Created by jakub on 24. 4. 2020.
//

#include <map>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


geometry_msgs::Twist currentTwist;

void cmdVelCb(const geometry_msgs::TwistPtr& msg) {
//    ROS_ERROR("RECV: %f, %f, %f", msg->linear.x, msg->linear.y, msg->angular.z);
    currentTwist = *msg;
}

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "virtual_robot" );
    ros::NodeHandle nh("~");

    currentTwist.angular.z = 0;
    currentTwist.linear.x = 0;
    currentTwist.linear.y = 0;
    ros::Subscriber cmdVelSub = nh.subscribe(ros::this_node::getNamespace() + "/mobile_base/commands/velocity", 5, cmdVelCb);

    std::string tfPrefix;
    nh.getParam("tf_prefix", tfPrefix);
    std::string odomFrame = tf::resolve(tfPrefix, "odom");
    std::string baseLinkFrame = tf::resolve(tfPrefix, "base_link");

    // read init pose
    std::map<std::string, double> initPose;
    nh.getParam("init_pose", initPose);

    geometry_msgs::TransformStamped odomTf;
    odomTf.header.stamp = ros::Time::now();
    odomTf.header.frame_id = "/map";
    odomTf.child_frame_id = odomFrame;
    odomTf.transform.translation.x = initPose["x"];
    odomTf.transform.translation.y = initPose["y"];
    tf::Quaternion qt;
    qt.setEuler(0, 0, initPose["a"]);
    odomTf.transform.rotation.w = qt.w();
    odomTf.transform.rotation.x = qt.x();
    odomTf.transform.rotation.y = qt.y();
    odomTf.transform.rotation.z = qt.z();
    tf2_ros::StaticTransformBroadcaster odomBroatcaster;
    odomBroatcaster.sendTransform(odomTf);


    tf::TransformBroadcaster baseLinkBroatcaster;
    tf::Transform baseLinkTf;

    double x = 0.0, y = 0.0, fi = 0.0;
    double periodTime = 0.1;
    ros::Rate r((ros::Duration(periodTime)));
    while (ros::ok()) {
//        ROS_ERROR("X = %f; Y = %f; FI= %f", x, y, fi);

        ros::spinOnce();

        fi = fi + currentTwist.angular.z * periodTime;
        // Keep orient_ between -pi and +pi
        fi -= 2*M_PI * std::floor((fi + M_PI)/(2*M_PI));
        x += std::cos(fi) * currentTwist.linear.x * periodTime;
        y += std::sin(fi) * currentTwist.linear.x * periodTime;

        baseLinkTf.setOrigin(tf::Vector3(x, y, 0));
        tf::Quaternion q;
        q.setRPY(0, 0, fi);
        baseLinkTf.setRotation(q);
        baseLinkBroatcaster.sendTransform(tf::StampedTransform(baseLinkTf, ros::Time::now(), odomFrame, baseLinkFrame));

        r.sleep();
    }

    return 0;
}
