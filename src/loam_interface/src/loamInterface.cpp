#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;

// Using unique_ptr or clearing inside class is cleaner, but keeping your global structure intact:
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
nav_msgs::msg::Odometry odomData;
tf2::Stamped<tf2::Transform> odomTrans;
geometry_msgs::msg::TransformStamped transformTfGeom; 

class LoamInterface : public rclcpp::Node
{
public:
  LoamInterface()
  : Node("loamInterface")
  {
    // 1. Declare Parameters with Defaults
    this->declare_parameter<std::string>("stateEstimationTopic", stateEstimationTopic);
    this->declare_parameter<std::string>("registeredScanTopic", registeredScanTopic);
    this->declare_parameter<std::string>("outputOdometryTopic", outputOdometryTopic);
    this->declare_parameter<std::string>("outputPointCloudTopic", outputPointCloudTopic);
    this->declare_parameter<std::string>("parentFrameId", parentFrameId);
    this->declare_parameter<std::string>("childFrameId", childFrameId);
    this->declare_parameter<bool>("flipStateEstimation", flipStateEstimation);
    this->declare_parameter<bool>("flipRegisteredScan", flipRegisteredScan);
    this->declare_parameter<bool>("sendTF", sendTF);
    this->declare_parameter<bool>("reverseTF", reverseTF);

    // 2. Fetch Parameter Values from YAML Configuration
    this->get_parameter("stateEstimationTopic", stateEstimationTopic);
    this->get_parameter("registeredScanTopic", registeredScanTopic);
    this->get_parameter("outputOdometryTopic", outputOdometryTopic);
    this->get_parameter("outputPointCloudTopic", outputPointCloudTopic);
    this->get_parameter("parentFrameId", parentFrameId);
    this->get_parameter("childFrameId", childFrameId);
    this->get_parameter("flipStateEstimation", flipStateEstimation);
    this->get_parameter("flipRegisteredScan", flipRegisteredScan);
    this->get_parameter("sendTF", sendTF);
    this->get_parameter("reverseTF", reverseTF);

    // 3. Initialize Dynamic Publishers and Subscriptions
    tfBroadcasterPointer = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    pubLaserCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(outputPointCloudTopic, 5);
    pubOdometry = this->create_publisher<nav_msgs::msg::Odometry>(outputOdometryTopic, 5);

    subLaserCloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      registeredScanTopic, 5, std::bind(&LoamInterface::laserCloudHandler, this, std::placeholders::_1));
    subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      stateEstimationTopic, 5, std::bind(&LoamInterface::odometryHandler, this, std::placeholders::_1));
  }

private:

  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn) const
  {
    laserCloud->clear();
    pcl::fromROSMsg(*laserCloudIn, *laserCloud);

    if (flipRegisteredScan) {
      int laserCloudSize = laserCloud->points.size();
      for (int i = 0; i < laserCloudSize; i++) {
        float temp = laserCloud->points[i].x;
        laserCloud->points[i].x = laserCloud->points[i].z;
        laserCloud->points[i].z = laserCloud->points[i].y;
        laserCloud->points[i].y = temp;
      }
    }

    // Publish registered scan messages using parameter frames
    sensor_msgs::msg::PointCloud2 laserCloud2;
    pcl::toROSMsg(*laserCloud, laserCloud2);
    laserCloud2.header.stamp = laserCloudIn->header.stamp;
    laserCloud2.header.frame_id = parentFrameId; // Configured via YAML
    pubLaserCloud->publish(laserCloud2);
  }

  void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odom) const
  {
    double roll, pitch, yaw;
    geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
    odomData = *odom;

    if (flipStateEstimation) {
      tf2::Matrix3x3(tf2::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

      pitch = -pitch;
      yaw = -yaw;

      tf2::Quaternion quat_tf;
      quat_tf.setRPY(roll, pitch, yaw);
      tf2::convert(quat_tf, geoQuat);

      odomData.pose.pose.orientation = geoQuat;
      odomData.pose.pose.position.x = odom->pose.pose.position.z;
      odomData.pose.pose.position.y = odom->pose.pose.position.x;
      odomData.pose.pose.position.z = odom->pose.pose.position.y;
    }

    // Publish corrected odometry frames
    odomData.header.frame_id = parentFrameId;
    odomData.child_frame_id = childFrameId;
    pubOdometry->publish(odomData);

    // Process dynamic transforms
    odomTrans.frame_id_ = parentFrameId;
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

    if (sendTF) {
      if (!reverseTF) {
        transformTfGeom = tf2::toMsg(odomTrans);
        transformTfGeom.child_frame_id = childFrameId;
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      } 
      else {
        transformTfGeom.transform = tf2::toMsg(odomTrans.inverse());
        transformTfGeom.header.frame_id = childFrameId;
        transformTfGeom.child_frame_id = parentFrameId;
        transformTfGeom.header.stamp = odom->header.stamp;
        tfBroadcasterPointer->sendTransform(transformTfGeom);
      }
    }
  }

  // Publishers and Subscribers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPointer;

  const double PI = 3.1415926;

  // Configuration parameter variables with baseline defaults
  string stateEstimationTopic = "/integrated_to_init";
  string registeredScanTopic = "/velodyne_cloud_registered";
  string outputOdometryTopic = "/state_estimation";
  string outputPointCloudTopic = "/registered_scan";
  string parentFrameId = "map";
  string childFrameId = "sensor";
  
  bool flipStateEstimation = true;
  bool flipRegisteredScan = true;
  bool sendTF = true;
  bool reverseTF = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoamInterface>());
  rclcpp::shutdown();
  return 0;
}