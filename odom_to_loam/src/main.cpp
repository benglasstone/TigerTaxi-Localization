#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>

using namespace std;

geometry_msgs::TransformStamped map_to_loam_init;
geometry_msgs::TransformStamped quatRot;
tf::Quaternion initialImu;

ros::Subscriber imuSub;
ros::Publisher loamPub;

void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg);
//void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg); //Used to test transformation when EKF is disabled

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "odom_to_loam");
	ros::NodeHandle n;

    auto odomSub = n.subscribe<nav_msgs::Odometry>("/ekf/loam_odom", 2, OdomCallback);
    imuSub = n.subscribe<sensor_msgs::Imu>("/vectornav/IMU", 2, imuCallback);

    //auto poseSubTest = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/laser_odom_map", 2, PoseCallback);
    loamPub = n.advertise<nav_msgs::Odometry>("/ekf/loam_converted_odom_gps_corrected", 5);

    ros::spin();
    return 0;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr &imuMsg)
{
    //printf("IMU Handler");
    initialImu = tf::Quaternion(0, 0, imuMsg->orientation.z, imuMsg->orientation.w);
    //ROS_INFO("IMU: %lf %lf %lf %lf", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
    imuSub.shutdown();
}

//Converts from ROS frame back to LOAM frame
void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::PoseStamped msgTransformTemp; //Create a PoseStamped to transform
    msgTransformTemp.header = msg->header;
    msgTransformTemp.pose.orientation = msg->pose.pose.orientation;
    msgTransformTemp.pose.position = msg->pose.pose.position;

    geometry_msgs::TransformStamped quatRot;
    quatRot.transform.translation.x = 0;
    quatRot.transform.translation.y = 0;
    quatRot.transform.translation.z = 0;
    quatRot.transform.rotation.x = initialImu.x();
    quatRot.transform.rotation.y = initialImu.y();
    quatRot.transform.rotation.z = initialImu.z();
    quatRot.transform.rotation.w = initialImu.w();

    //Transform based on initial IMU
    tf2::doTransform(msgTransformTemp, msgTransformTemp, quatRot);

    nav_msgs::Odometry loamMsg;
    loamMsg.header = msg->header;
    loamMsg.header.frame_id = "loam_odom";
    loamMsg.pose.pose.position.x = msgTransformTemp.pose.position.y;
    loamMsg.pose.pose.position.y = msgTransformTemp.pose.position.z;
    loamMsg.pose.pose.position.z = msgTransformTemp.pose.position.x;
    loamMsg.pose.pose.orientation.x = msgTransformTemp.pose.orientation.y;
    loamMsg.pose.pose.orientation.y = msgTransformTemp.pose.orientation.z;
    loamMsg.pose.pose.orientation.z = msgTransformTemp.pose.orientation.x;
    loamMsg.pose.pose.orientation.w = msgTransformTemp.pose.orientation.w;


    loamPub.publish(loamMsg);
}

/*void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    ROS_INFO_STREAM("No EKF mode");
    geometry_msgs::PoseStamped msgTransformTemp, msgQuaternionTemp;
    msgQuaternionTemp.pose.orientation = msg->pose.pose.orientation;
    tf2::doTransform(msgQuaternionTemp, msgQuaternionTemp, quatRot);
    msgTransformTemp.pose.orientation = msgQuaternionTemp.pose.orientation;
    msgTransformTemp.pose.position = msg->pose.pose.position;
    tf2::doTransform(msgTransformTemp, msgTransformTemp, map_to_loam_init); //Internally, LOAM has data in loam_init frame. Therefore, we need to transform it to map frame.
    geometry_msgs::PoseWithCovarianceStamped loamMsg;
    loamMsg.header = msg->header;
    loamMsg.header.frame_id = "loam_init";
    loamMsg.pose.pose.position = msgTransformTemp.pose.position;
    loamMsg.pose.pose.orientation = msgTransformTemp.pose.orientation;
    loamPub.publish(loamMsg);
}*/
