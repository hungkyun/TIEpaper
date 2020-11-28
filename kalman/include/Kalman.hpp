#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <cmath>
#include <vector>
#include <ctime>
#include <sstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>  

class Tracker{
public:
	Tracker();
	void clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,std::vector<pcl::PointIndices> &cluster_indices);
	void processPointcloud(/*pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud*/const sensor_msgs::PointCloud2 &scan);
	void on_HoughLines(const cv::Mat &a, std::vector<cv::Vec4i> &lines);
	void TransformCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr hough,pcl::PointCloud<pcl::PointXYZI>::Ptr new1);
	void readPCDfile(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher raw_pub;
	ros::Publisher out_pub;
};

Tracker::Tracker()
{
	// Tracker::processPointcloud(cloud)
	points_sub = nh.subscribe("velodyne_points", 1028, &Tracker::processPointcloud, this);
	raw_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_raw", 10);
	out_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_out", 10);
}
