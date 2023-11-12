#include "apriltag_opencv.h"
#include "apriltag_family.h"
#include "getopt.h"
#include "homography.h"
#include "pose.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

const double fx=279.3362, fy=281.7544, cx=331.7082, cy=242.6293, // Camera parameters
         z_sign=1.0;//0.062
double tagsize;
Eigen::Matrix3f extrin_rot;
Eigen::Vector3f extrin_trans;

const char* MAT_FMT = "\t%12f, ";

// const char* window = "AprilTag";

cv::Mat frame;

apriltag_detector_t *td;
ros::Publisher odo_pub, cloud_pub, tag_cloud_pub;
void callback(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
    cv::Mat frame = cv_ptr->image;
    // cv::imshow(window, frame);

    Mat8uc1 gray;

    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
    }
    else {
        frame.copyTo(gray);
    }

    image_u8_t* im8 = cv2im8_copy(gray);

    zarray_t *detections = apriltag_detector_detect(td, im8);//! jin:已知图象中点和实际标定板中点的对应关系，可以计算H矩阵

    printf("Detected %d tags.\n", zarray_size(detections));

    // cv::Mat display = detectionsImage(detections, frame.size(), frame.type());

    pcl::PointCloud<pcl::PointXYZINormal> cloud;

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        matd_t* M = pose_from_homography(det->H, fx, fy, cx, cy,
                                         tagsize, z_sign, det->p, NULL, NULL);//! jin:已知H矩阵，可以计算出带尺度的Rt，已知码的实际大小，可以恢复出真实pose,tag码的中心，右手坐标系

        printf("Detection %d of %d:\n \tFamily: tag%2dh%2d\n \tID: %d\n \tHamming: %d\n"
               "\tGoodness: %.3f\n \tMargin: %.3f\n \tCenter: (%.3f,%.3f)\n"
               "\tCorners: (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n",
               i+1, zarray_size(detections), det->family->d*det->family->d, det->family->h,
               det->id, det->hamming, det->goodness, det->decision_margin, det->c[0], det->c[1],
               det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1],
               det->p[3][0], det->p[3][1]);
        printf("\tHomography:\n");
        matd_print(det->H, MAT_FMT);
        printf("\tPose:\n");
        matd_print(M, MAT_FMT);

        pcl::PointXYZINormal p;
        Eigen::Vector3f trans_cam((M)->data[0*4+3], (M)->data[1*4+3], (M)->data[2*4+3]);
        Eigen::Vector3f trans_lidar = extrin_rot * trans_cam + extrin_trans;
        p.x = trans_lidar.x();
        p.y = trans_lidar.y();
        p.z = trans_lidar.z();
        Eigen::Matrix3f rot;
        rot << (M)->data[0*4+0], (M)->data[0*4+1], (M)->data[0*4+2],
                (M)->data[1*4+0], (M)->data[1*4+1], (M)->data[1*4+2],
                (M)->data[2*4+0], (M)->data[2*4+1], (M)->data[2*4+2];
        rot = extrin_rot * rot;
        Eigen::AngleAxisf q(rot);
        Eigen::Vector3f direction = q.axis();
        float norm = q.angle();
        Eigen::Vector3f rotation = direction * norm;
        p.normal_x = rotation.x();
        p.normal_y = rotation.y();
        p.normal_z = rotation.z();
        p.intensity = det->id;
        cloud.push_back(p);

    }
    cloud.width = cloud.points.size();
    if(cloud.points.size() > 0){
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(cloud, ros_cloud);
      ros_cloud.header.frame_id = "lidar_frame";  
      ros_cloud.header.stamp = msg->header.stamp;
      tag_cloud_pub.publish(ros_cloud);
    }
    

    printf("\n");

    apriltag_detections_destroy(detections);

    image_u8_destroy(im8);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_video");
  ros::NodeHandle nh;

  extrin_rot << 0.0, 0.0, 1.0,
          -1.0, 0.0, 0.0,
          0.0, -1.0, 0.0;
  extrin_trans << 0.08, 0.0, -0.07;
  
  nh.param<double>("tag_size", tagsize, 0.1);

  ROS_INFO("tag_size = %f", tagsize);
  
  getopt_t *getopt = getopt_create();

  getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
  getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
  getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
  getopt_add_int(getopt, '\0', "border", "1", "Set tag family border size");
  getopt_add_int(getopt, 't', "threads", "4", "Use this many CPU threads");
  getopt_add_double(getopt, 'x', "decimate", "1.0", "Decimate input image by this factor");
  getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
  getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time aligning edges of tags");
  getopt_add_bool(getopt, '1', "refine-decode", 0, "Spend more time decoding tags");
  getopt_add_bool(getopt, '2', "refine-pose", 0, "Spend more time computing pose of tags");
  getopt_add_bool(getopt, 'c', "contours", 0, "Use new contour-based quad detection");

  if (!getopt_parse(getopt, argc, argv, 1) || getopt_get_bool(getopt, "help")) {
    printf("Usage: %s [options] <camera index or path to movie file>\n", argv[0]);
    getopt_do_usage(getopt);
    exit(0);
  }

  const char *famname = getopt_get_string(getopt, "family");
  apriltag_family_t *tf = apriltag_family_create(famname);

  if (!tf) {
    printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
    exit(-1);
  }

  tf->black_border = getopt_get_int(getopt, "border");

  td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);

  if (getopt_get_bool(getopt, "contours")) {
    apriltag_detector_enable_quad_contours(td, 1);
  }

  td->quad_decimate = getopt_get_double(getopt, "decimate");
  td->quad_sigma = getopt_get_double(getopt, "blur");
  td->nthreads = getopt_get_int(getopt, "threads");
  td->debug = 0;
  td->refine_edges = getopt_get_bool(getopt, "refine-edges");
  td->refine_decode = getopt_get_bool(getopt, "refine-decode");
  td->refine_pose = getopt_get_bool(getopt, "refine-pose");

  const zarray_t *inputs = getopt_get_extra_args(getopt);

  int camera_index = 0;
  const char* movie_file = NULL;

  if (zarray_size(inputs) > 1) {
    printf("Usage: %s [options] <camera index or path to movie file>\n", argv[0]);
    exit(-1);
  }
  else if (zarray_size(inputs)) {
    char* input;
    zarray_get(inputs, 0, &input);
    char* endptr;
    camera_index = strtol(input, &endptr, 10);
    if (!endptr || *endptr) {
      movie_file = input;
    }
  }

    // cv::namedWindow(window);
    // odo_pub = nh.advertise<nav_msgs::Odometry>("/tag_pose", 5);
    // cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/scans", 5);


    tag_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/tag_pose_cloud", 5);

    ros::Subscriber img_sub = nh.subscribe<sensor_msgs::Image>("/usb_cam/image_raw", 5, &callback);
    // ros::Subscriber cloud_sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 5, &livoxPointCloudCallback);

    ros::spin();
//  cv::VideoCapture* cap;
//
//  if (movie_file) {
//    cap = new cv::VideoCapture(movie_file);
//  }
//  else {
//    cap = new cv::VideoCapture(camera_index);
//  }

  // const double fx=3156.71852, fy=3129.52243, cx=359.097908, cy=239.736909, // Camera parameters
  //              tagsize=0.0762, z_sign=1.0;
//  const double fx=279.3362, fy=281.7544, cx=331.7082, cy=242.6293, // Camera parameters
//               tagsize=0.062, z_sign=1.0;
//
//  const char* MAT_FMT = "\t%12f, ";
//
//  const char* window = "AprilTag";
//
//  cv::Mat frame;
//
//  cv::namedWindow(window);

//  while (1) {
//    bool ok = cap->read(frame);
//    if (!ok) { break; }
//    cv::imshow(window, frame);
//
//    Mat8uc1 gray;
//
//    if (frame.channels() == 3) {
//      cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
//    }
//    else {
//      frame.copyTo(gray);
//    }
//
//    image_u8_t* im8 = cv2im8_copy(gray);
//
//    zarray_t *detections = apriltag_detector_detect(td, im8);//! jin:已知图象中点和实际标定板中点的对应关系，可以计算H矩阵
//
//    printf("Detected %d tags.\n", zarray_size(detections));
//
//    cv::Mat display = detectionsImage(detections, frame.size(), frame.type());
//
//    for (int i = 0; i < zarray_size(detections); i++) {
//      apriltag_detection_t *det;
//      zarray_get(detections, i, &det);
//
//      matd_t* M = pose_from_homography(det->H, fx, fy, cx, cy,
//                                       tagsize, z_sign, det->p, NULL, NULL);//! jin:已知H矩阵，可以计算出带尺度的Rt，已知码的实际大小，可以恢复出真实pose,tag码的中心，右手坐标系
//
//      printf("Detection %d of %d:\n \tFamily: tag%2dh%2d\n \tID: %d\n \tHamming: %d\n"
//             "\tGoodness: %.3f\n \tMargin: %.3f\n \tCenter: (%.3f,%.3f)\n"
//             "\tCorners: (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n\t\t (%.3f,%.3f)\n",
//             i+1, zarray_size(detections), det->family->d*det->family->d, det->family->h,
//             det->id, det->hamming, det->goodness, det->decision_margin, det->c[0], det->c[1],
//             det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1], det->p[2][0], det->p[2][1],
//             det->p[3][0], det->p[3][1]);
//      printf("\tHomography:\n");
//      matd_print(det->H, MAT_FMT);
//      printf("\tPose:\n");
//      matd_print(M, MAT_FMT);
//    }
//
//    printf("\n");
//
//    apriltag_detections_destroy(detections);
//
//    display = 0.5*display + 0.5*frame;
//    cv::imshow(window, display);
//    image_u8_destroy(im8);
//
//    int k = cv::waitKey(1);
//    if (k == 27) { break; }
//
//  }

  return 0;

}
