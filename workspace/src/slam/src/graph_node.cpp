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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

int T = 100;

int next_position = 1000;
bool conect_pose = false;

std::vector<float> old_position(3, 0);
std::vector<float> current_position(3, 0);

std::ofstream oFile("graph.txt", std::ios_base::out | std::ios_base::trunc);

//tf2_ros::Buffer tfBuffer;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom)
{
    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3 (q).getRPY(roll, pitch, yaw);

    current_position[0] = odom->pose.pose.position.x;
    current_position[1] = odom->pose.pose.position.y;
    current_position[2] = yaw;

    float dx = current_position[0] - old_position[0];
    float dy = current_position[1] - old_position[1];
    float dth = current_position[2] - old_position[2];

    float distance = sqrt(pow(dx, 2) + pow(dy, 2));


    if (distance > 0.1 || dth > 0.5 || !conect_pose)
    {
        old_position[0] = current_position[0];
        old_position[1] = current_position[1];
        old_position[2] = current_position[2];

        if (oFile.is_open())
        {
            oFile << "VERTEX_SE2"
                  << " " << next_position
                  << " " << current_position[0]
                  << " " << current_position[1]
                  << " " << current_position[2]
                  << "\n";

            if (conect_pose)
            {
                oFile << "EDGE_SE2"
                      << " " << next_position - 1
                      << " " << next_position
                      << " " << dx
                      << " " << dy
                      << " " << dth
                      << "\n";
            }
            next_position++;
            conect_pose = true;
        }
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_tf = nh.subscribe("/odom", 1000, odometryCallback);

    //tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate loop_rate(T);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    oFile.close();
    return 0;
}
