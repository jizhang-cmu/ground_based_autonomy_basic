#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double sensorOffsetX = 0;
double sensorOffsetY = 0;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());

ros::Time odomTime;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0.75;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

ros::Publisher *pubScanPointer = NULL;

void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn)
{
  double scanTime = scanIn->header.stamp.toSec();

  if (odomSendIDPointer < 0) {
    return;
  }
  while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime && 
         odomRecIDPointer != (odomSendIDPointer + 1) % stackNum) {
    odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
  }

  double odomRecTime = odomTime.toSec();
  float vehicleRecX = vehicleX;
  float vehicleRecY = vehicleY;
  float vehicleRecZ = vehicleZ;
  float vehicleRecRoll = vehicleRoll;
  float vehicleRecPitch = vehiclePitch;
  float vehicleRecYaw = vehicleYaw;

  if (use_gazebo_time) {
    odomRecTime = odomTimeStack[odomRecIDPointer];
    vehicleRecX = vehicleXStack[odomRecIDPointer];
    vehicleRecY = vehicleYStack[odomRecIDPointer];
    vehicleRecZ = vehicleZStack[odomRecIDPointer];
    vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
    vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
    vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
  }

  scanData->clear();
  pcl::fromROSMsg(*scanIn, *scanData);

  int scanDataSize = scanData->points.size();
  for (int i = 0; i < scanDataSize; i++) {
    float pointX = scanData->points[i].x + vehicleRecX;
    float pointY = scanData->points[i].y + vehicleRecY;
    float pointZ = scanData->points[i].z + vehicleRecZ;

    scanData->points[i].x = pointX;
    scanData->points[i].y = pointY;
    scanData->points[i].z = pointZ;
  }

  // publish 5Hz registered scan messages
  sensor_msgs::PointCloud2 scanData2;
  pcl::toROSMsg(*scanData, scanData2);
  scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
  scanData2.header.frame_id = "/map";
  pubScanPointer->publish(scanData2);
}

void speedHandler(const geometry_msgs::TwistStamped::ConstPtr& speedIn)
{
  vehicleSpeed = speedIn->twist.linear.x;
  vehicleYawRate = speedIn->twist.angular.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
  nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
  nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
  nhPrivate.getParam("vehicleZ", vehicleZ);

  ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>
                            ("/velodyne_points", 2, scanHandler);

  ros::Subscriber subSpeed = nh.subscribe<geometry_msgs::TwistStamped>
                            ("/cmd_vel", 5, speedHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "/map";
  odomData.child_frame_id = "/sensor";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform odomTrans;
  odomTrans.frame_id_ = "/map";
  odomTrans.child_frame_id_ = "/sensor";

  ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 5);
  gazebo_msgs::ModelState stateData;
  stateData.model_name = "sensor";

  ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 2);
  pubScanPointer = &pubScan;

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    vehicleYaw += 0.005 * vehicleYawRate;
    if (vehicleYaw > PI) vehicleYaw -= 2 * PI;
    else if (vehicleYaw < -PI) vehicleYaw += 2 * PI;

    vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed 
              + 0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
    vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed 
              + 0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);

    odomTime = ros::Time::now();

    odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
    odomTimeStack[odomSendIDPointer] = odomTime.toSec();
    vehicleXStack[odomSendIDPointer] = vehicleX;
    vehicleYStack[odomSendIDPointer] = vehicleY;
    vehicleZStack[odomSendIDPointer] = vehicleZ;
    vehicleRollStack[odomSendIDPointer] = vehicleRoll;
    vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
    vehicleYawStack[odomSendIDPointer] = vehicleYaw;

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    // publish 200Hz odometry messages
    odomData.header.stamp = odomTime;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleSpeed;
    pubVehicleOdom.publish(odomData);

    // publish 200Hz tf messages
    odomTrans.stamp_ = odomTime;
    odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    tfBroadcaster.sendTransform(odomTrans);

    // publish 200Hz Gazebo model state messages (this is for Gazebo simulation)
    stateData.pose.position.x = vehicleX;
    stateData.pose.position.y = vehicleY;
    stateData.pose.position.z = vehicleZ;
    pubModelState.publish(stateData);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
