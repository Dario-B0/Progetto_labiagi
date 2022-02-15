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
#include "sensor_msgs/LaserScan.h"

using namespace std;

int T = 100;

int next_position = 1000;
int next_view_position = 1000;
bool conect_pose = false;
bool first_detect = true;
bool started = false;
bool stop = false;

sensor_msgs::LaserScan last_scan;

//ultima posizione del robot aggiunta al grafo
vector<float> last_pose = {0, 0, 0, 0, 0, 0};
ros::Time last_time;

//posizione attuale del robot
vector<float> current_pose = {0, 0, 0, 0, 0, 0};
ros::Time current_time;

map<int, set<int>> tag_in;

ofstream oFile("graph.g2o", ios_base::out | ios_base::trunc);

tf2_ros::Buffer tfBuffer;

tf2::Transform robot_camera_T;

void odometryCallback(const nav_msgs::OdometryConstPtr &odom)
{
    started = true;
    stop = false;
    tf2::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_pose[0] = odom->pose.pose.position.x;
    current_pose[1] = odom->pose.pose.position.y;
    current_pose[2] = odom->pose.pose.position.z;
    current_pose[3] = roll;
    current_pose[4] = pitch;
    current_pose[5] = yaw;

    current_time = odom->header.stamp;

    float dx = current_pose[0] - last_pose[0];
    float dy = current_pose[1] - last_pose[1];
    float dth = current_pose[5] - last_pose[5];

    float distance = sqrt(pow(dx, 2) + pow(dy, 2));

    if (distance > 0.1 || dth > 0.5 || !conect_pose)
    {
        last_pose[0] = current_pose[0];
        last_pose[1] = current_pose[1];
        last_pose[2] = current_pose[2];
        last_pose[3] = current_pose[3];
        last_pose[4] = current_pose[4];
        last_pose[5] = current_pose[5];

        last_time = current_time;

        if (oFile.is_open())
        {
            oFile << "VERTEX_SE2"
                  << " " << next_position
                  << " " << current_pose[0]
                  << " " << current_pose[1]
                  << " " << current_pose[5]
                  << "\n";

            if (conect_pose)
            {
                oFile << "EDGE_SE2"
                      << " " << next_position - 1
                      << " " << next_position
                      << " " << dx
                      << " " << dy
                      << " " << dth
                      << " 500 0 0 500 0 5000"
                      << "\n";
            }
            next_position++;
            conect_pose = true;
        }
    }
    return;
}

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &detection_msg)
{
    stop = false;
    if (detection_msg->detections.size() == 0)
        return;
    if (first_detect)
    {
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            //da base link a fisheye rect c'è solo una trasformazione statica quindi
            //da calcolare solo una volta
            transformStamped = tfBuffer.lookupTransform("base_link", "fisheye_rect", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            return;
        }
        tf2::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        tf2::Vector3 v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
        robot_camera_T = tf2::Transform(q, v);
        first_detect = false;
    }

    tf2::Quaternion q;
    q.setRPY(current_pose[3] - last_pose[3], current_pose[4] - last_pose[4], current_pose[5] - last_pose[5]);
    tf2::Vector3 v(current_pose[0] - last_pose[0], current_pose[1] - last_pose[1], current_pose[2] - last_pose[2]);
    tf2::Transform T(q, v); //trasformazione tra la poszione corrente del robot e l'ultima aggiunta al grafo

    int tag_id;

    for (const apriltag_ros::AprilTagDetection &tag : detection_msg->detections)
    {
        tag_id = tag.id[0];

        //trasformazione dal frame della camera al frame dell'odometria

        bool new_tag = tag_in.find(tag_id) == tag_in.end();

        if (new_tag || tag_in[tag_id].find(next_position) == tag_in[tag_id].end())
        {

            tf2::Vector3 tag_position_cf(tag.pose.pose.pose.position.x, tag.pose.pose.pose.position.y, tag.pose.pose.pose.position.z);
            tf2::Vector3 tag_position_rf_now = robot_camera_T * tag_position_cf;

            float theta_tag = atan2(tag_position_rf_now.y(), tag_position_rf_now.x());
            float theta_range = last_scan.angle_min;
            std::vector<float> ranges = last_scan.ranges;

            int k = 0;
            while (k < ranges.size())
            {
                if (theta_range > theta_tag)
                {
                    break;
                }
                theta_range += last_scan.angle_increment;
                k++;
            }
            if(k==ranges.size() || theta_range-theta_tag>theta_tag-(theta_range-last_scan.angle_increment)) k--;
            float scan_distance = ranges[k];
            float tag_distance = sqrt(pow(tag_position_rf_now.x(), 2) + pow(tag_position_rf_now.y(), 2));
            float error = scan_distance - tag_distance;

            tf2::Vector3 tag_position_rf = T * tag_position_rf_now;

            if (error < -0.5 || error > 0.5) continue;

            /*if (error < -0.5 || error > 0.5 || tag_position_rf.z()<-0.2 || tag_position_rf.z()>0.5 ||tag_id>9)
            {
                cerr <<endl<< tag_id <<" : discrepanza tra laser scan e tag detection: " << error ;
                //continue;
            }

            if(tag_position_rf.z()<-0.2 || tag_position_rf.z()>0.5 || tag_id>9) {
                cerr << "  TAG ERRATO";
                //continue;
                }*/

            if (new_tag)
            {
            geometry_msgs::TransformStamped transformStamped;
            //geometry_msgs::TransformStamped transformStamped2;
            try
            {
                transformStamped = tfBuffer.lookupTransform("odom", "fisheye_rect", current_time, ros::Duration(3));
                //transformStamped2 = tfBuffer.lookupTransform("odom", "laser_frame", last_scan.header.stamp, ros::Duration(3));
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                return;
            }

            set<int> map_set{};
            tag_in[tag_id] = map_set;

            //trasformazione dal frame della camera al frame dell'odometria
            tf2::Quaternion q2;
            q2 = tf2::Quaternion(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
            tf2::Vector3 v2 = tf2::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
            tf2::Transform T2 = tf2::Transform(q2, v2);
            tf2::Vector3 tag_position_wf = T2 * tag_position_cf;

            /////
            /*tf2::Quaternion q3;
            q3= tf2::Quaternion(transformStamped2.transform.rotation.x, transformStamped2.transform.rotation.y, transformStamped2.transform.rotation.z, transformStamped2.transform.rotation.w);
            tf2::Vector3 v3 = tf2::Vector3(transformStamped2.transform.translation.x, transformStamped2.transform.translation.y, transformStamped2.transform.translation.z);
            tf2::Transform T3 = tf2::Transform(q3, v3);*/

            ////

            //cerr << "\ntag_id: "<<tag_id<<"\nworld " << tag_position_wf.x()  << " " << tag_position_wf.y() <<" " << tag_position_wf.z();
            //cerr << "\nworld2 " << tag_position_wf_alt.x() << " " << tag_position_wf_alt.y()<< " " << tag_position_wf_alt.z();

            //aggiungi nodo VERTEX_XY <xw> <yw> al grafo
            oFile << "VERTEX_XY"
                      << " " << tag_id
                      << " " << tag_position_wf.x() << " " << tag_position_wf.y() << "\n";
            }

            //aggiungi arco EDGE_SE2_XY <id_pose> <id_tag> <xr> <yr> al grafo
            oFile << "EDGE_SE2_XY"
                  << " " << next_position - 1 << " " << tag_id
                  << " " << tag_position_rf.x() << " " << tag_position_rf.y() << " 1000 0 1000"
                  << "\n";
            //float distance=sqrt(pow(tag_position_rf.x(),2)+pow(tag_position_rf.y(),2)+pow(tag_position_rf.z(),2));
            tag_in[tag_id].insert(next_position);
            //cerr << "\ncamera " << tag_position_cf.x() << " " << tag_position_cf.y() << " " << tag_position_cf.z()
            //cerr << "\nrobot "<< tag_position_rf.x() << " " << tag_position_rf.y()  <<" "<< tag_position_rf.z() <<"  "<<next_position <<" distance: "<<distance<< endl;
        }
    }
    return;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg_)
{
    last_scan = *msg_;
}

void check_Callback(const ros::TimerEvent &event)
{
    if (started && stop)
    {
        oFile.close();
        cerr << "\nENDING PROCEDURE" << endl;
        exit(0);
    }
    stop = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "graph_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_tf = nh.subscribe("/odom", 1000, odometryCallback);
    ros::Subscriber sub_tag_detc = nh.subscribe("/tag_detections", 1000, tagDetectionsCallback);
    ros::Subscriber sub_scan = nh.subscribe("/base_scan", 1000, scanCallback);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Timer timer1 = nh.createTimer(ros::Duration(5), check_Callback);

    ros::spin();

    return 0;
}
