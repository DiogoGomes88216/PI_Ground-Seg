#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "groundSeg.h"

#define grid_size 1.0f

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create data.
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;

    //Convert Point Cloud 
    pcl::fromROSMsg(*input, *cloud);

    //Apply Filter
    pcl::GroundSeg gs(grid_size);
    gs.setInputCloud(cloud);
    gs.filter(cloud_filtered);

    // Back to sensor_msgs
    pcl::toROSMsg(cloud_filtered, output);

    // Publish the data.
    pub.publish (output);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "Ground_Segmentation");
    ros::NodeHandle nh;
    std::string topic = nh.resolveName("/autonomoose/velo/pointcloud");
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
}