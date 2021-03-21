#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/String.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/UIDfragSmart.hpp"

#include <iostream>
#include <cstdlib>



cv::Mat img;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw (new pcl::PointCloud<pcl::PointXYZRGB>);
UIDfragSmart* window;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(window->isLblImReady)
  {
    img = cv_bridge::toCvShare(msg, "bgr8")->image;
    window->isImageReady = true;
  }
}

void zedCloudHandler(const sensor_msgs::PointCloud2ConstPtr& zedCloudMsg)
{  
	//double timeScanCur = zedCloudMsg->header.stamp.toSec();
  //if(window->isQvtkCloudReady)
  //{
	  pcl::fromROSMsg(*zedCloudMsg, *zedCloudRaw);
    window->isPointcloudReady = true;
  //}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rock_metric");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/zed/zed_node/rgb/image_rect_color", 1, imageCallback);

  ros::Subscriber subZedCloud = nh.subscribe<sensor_msgs::PointCloud2>
		                        ("/zed/zed_node/point_cloud/cloud_registered", 2, zedCloudHandler);

  ros::Publisher pubGainExposure = nh.advertise<std_msgs::String>("/zed/zed_node/gain_exposure", 1);

  QApplication a (argc, argv);
  window = new UIDfragSmart(&pubGainExposure, &img, zedCloudRaw);
	window->show();
  int r = a.exec();

  delete window;
 
  return r;
}
