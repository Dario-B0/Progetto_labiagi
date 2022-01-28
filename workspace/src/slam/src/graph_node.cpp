#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "vector"
#include "tf/tf.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

int T = 100;
int next_position = 0;
std::ofstream oFile("graph.txt", std::ios_base::out | std::ios_base::trunc);

tf2_ros::Buffer tfBuffer;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom)
{

    if (oFile.is_open()){
            oFile << "VERTEX_SE2 " 
            << "<" << next_position << "> "
            << "<" << odom->pose.pose.position.x << "> "
            << "<" << odom->pose.pose.position.y << "> "
            << "<" << odom->pose.pose.orientation.w << "> "
            << "\n";
    next_position++;

    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_tf = nh.subscribe("/odom", 1000, odometryCallback);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(T);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    oFile.close();
    return 0;
}
