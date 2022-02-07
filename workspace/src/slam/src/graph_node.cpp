#include <fstream>
#include <iostream>
#include <string>
#include <ros/ros.h>
#include "vector"
#include "set"
#include "tf/tf.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2/convert.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

using namespace std;

int T = 100;

int next_position = 1000;
bool conect_pose = false;
bool first_detect = true;

//ultima posizione del robot aggiunta al grafo
vector<float> last_position(3, 0);
ros::Time last_time;

//posizione attuale del robot
vector<float> current_position(3, 0);
ros::Time current_time;

set<int> tag_in = {};

ofstream oFile("graph.txt", ios_base::out | ios_base::trunc);

tf2_ros::Buffer tfBuffer;

tf2::Transform robot_camera_T;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom)
{

    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_position[0] = odom->pose.pose.position.x;
    current_position[1] = odom->pose.pose.position.y;
    current_position[2] = yaw;
    current_time = odom->header.stamp;

    float dx = current_position[0] - last_position[0];
    float dy = current_position[1] - last_position[1];
    float dth = current_position[2] - last_position[2];

    float distance = sqrt(pow(dx, 2) + pow(dy, 2));

    if (distance > 0.1 || dth > 0.5 || !conect_pose)
    {
        last_position[0] = current_position[0];
        last_position[1] = current_position[1];
        last_position[2] = current_position[2];
        last_time = current_time;

        if (oFile.is_open())
        {
            /*oFile << "VERTEX_SE2"
                  << " " << next_position
                  << " " << current_position[0]
                  << " " << current_position[1]
                  << " " << current_position[2]
                  << "\n";*/

            if (conect_pose)
            {
                /*oFile << "EDGE_SE2"
                      << " " << next_position - 1
                      << " " << next_position
                      << " " << dx
                      << " " << dy
                      << " " << dth
                      << "\n";*/
            }
            next_position++;
            conect_pose = true;
        }
    }
    return;
}

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_msg)
{
    if (first_detect)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            //da base link a fisheye rect c'è solo una trasformazione statica
            transformStamped = tfBuffer.lookupTransform("base_link", "fisheye_rect", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        tf2::Vector3 v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        robot_camera_T = tf2::Transform(q, v);
        first_detect = false;
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, current_position[2] - last_position[2]);
    tf2::Vector3 v(current_position[0] - last_position[0], current_position[1] - last_position[1], 0);
    tf2::Transform T(q, v); //transformation from the current position to the last added to the graph

    int tag_id;
    float tag_size;

    for (const apriltag_ros::AprilTagDetection &tag : detection_msg->detections)
    {
        tag_id = tag.id[0];
        tag_size = tag.size[0];

        tf2::Vector3 tag_position_cf(tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z);

        tf2::Vector3 tag_position_rf = T * (robot_camera_T * tag_position_cf);

        //aggiungi gli archi al grafo

        if (tag_in.find(tag_id) == tag_in.end())
        {
            tag_in.insert(tag_id);
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped = tfBuffer.lookupTransform("odom", "fisheye_rect", detection_msg->header.stamp, ros::Duration(1));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            //trasformazione dal frame della camera al frame dell'odometria
            q = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            v = tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            T = tf2::Transform(q, v);
            tf2::Vector3 tag_position_wf = T * tag_position_cf;
            cerr << tag_id << "\nworld " << tag_position_wf.getX() << " " << tag_position_wf.getY() << endl;

            //aggiungi nodo al grafo
        }

        cerr << tag_id << "\ncamera " << tag_position_cf.getX() << " " << tag_position_cf.getY()
             << "\nrobot " << tag_position_rf.getX() << " " << tag_position_rf.getY() << endl
             << endl;
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_tf = nh.subscribe("/odom", 1000, odometryCallback);
    ros::Subscriber sub_tag_detc = nh.subscribe("/tag_detections", 1000, tagDetectionsCallback);

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
