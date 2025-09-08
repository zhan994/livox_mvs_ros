#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void CloudCBK(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(cloudPtr);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(1, 40);
  pass.filter(cloud_filtered);

  sensor_msgs::PointCloud2 cloud_pt;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_pt);

  pub.publish(cloud_pt);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "PassThroughX");
  ros::NodeHandle nh;
  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, CloudCBK);
  pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/filter_x", 1);
  ros::spin();

  return 0;
}