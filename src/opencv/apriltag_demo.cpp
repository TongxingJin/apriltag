#include "apriltag_opencv.h"
#include "apriltag_family.h"
#include "getopt.h"
#include "homography.h"
#include "pose.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

cv::Mat frame;

apriltag_detector_t *td;
ros::Publisher odo_pub, cloud_pub;
void callback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud);

    std::cout << "#########################Detected tag size: " << pcl_cloud->size() << std::endl;
    std::cout << "{ ";
    for(int index = 0; index < pcl_cloud->size(); ++index){
      std::cout << pcl_cloud->points[index].intensity << ", ";
    } 
    std::cout << " }" << std::endl;
    for (int i = 0; i < 3; i++) {
      if(i >= pcl_cloud->points.size()) continue;
      const pcl::PointXYZINormal& p = pcl_cloud->points[i];  
      nav_msgs::Odometry real_odo;
      real_odo.header.frame_id = "lidar_frame";
      real_odo.child_frame_id = "tag";
      real_odo.header.stamp = msg->header.stamp;// ros::Time().fromSec(lidar_end_time);
      real_odo.pose.pose.position.x = p.x;
      real_odo.pose.pose.position.y = p.y;
      real_odo.pose.pose.position.z = p.z;
      Eigen::Vector3f norm(p.normal_x, p.normal_y, p.normal_z);
      Eigen::AngleAxisf q(norm.norm(), norm.normalized());
      Eigen::Quaternionf qq(q);
      real_odo.pose.pose.orientation.x = qq.x();
      real_odo.pose.pose.orientation.y = qq.y();
      real_odo.pose.pose.orientation.z = qq.z();
      real_odo.pose.pose.orientation.w = qq.w();
      odo_pub.publish(real_odo);
    }
}


void livoxPointCloudCallback(const livox_ros_driver::CustomMsg::ConstPtr& livox_cloud_msg){
      pcl::PointCloud<pcl::PointXYZI> output_cloud;
      for (uint i = 0; i < livox_cloud_msg->point_num; ++i) {
        pcl::PointXYZI p;
        p.x = livox_cloud_msg->points[i].x;
        p.y = livox_cloud_msg->points[i].y;
        p.z = livox_cloud_msg->points[i].z;
        p.intensity = livox_cloud_msg->points[i].reflectivity;
        output_cloud.points.push_back(p);
      }
      sensor_msgs::PointCloud2 laserCloudmsg;
      pcl::toROSMsg(output_cloud, laserCloudmsg);
      laserCloudmsg.header.stamp = livox_cloud_msg->header.stamp;
      laserCloudmsg.header.frame_id = "lidar_frame";
      cloud_pub.publish(laserCloudmsg);
}
int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_demo");
  ros::NodeHandle nh;

    odo_pub = nh.advertise<nav_msgs::Odometry>("/tag_pose", 5);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/scans", 5);
    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::PointCloud2>("/tag_pose_cloud", 5, &callback);
    ros::Subscriber cloud_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 5, &livoxPointCloudCallback);

    ros::spin();

  return 0;

}
