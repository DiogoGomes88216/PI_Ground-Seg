#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "../include/groundSeg.h" //VSCODE nao reconhece "groundSeg.h" - só para tirar squiggle do VSCODE

#define grid_size 0.5f

ros::Publisher pub_ground, pub_non_ground;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create data.
    sensor_msgs::PointCloud2 out_ground,out_non_ground;
    pcl::PointCloud <pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    //pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::PointCloud <pcl::PointXYZ> non_ground, ground;

    //Convert Point Cloud 
    pcl::fromROSMsg(*input, *cloud);

    PCL_WARN("-------------START-----------------\n");
    //Apply Filter
    pcl::GroundSeg gs(grid_size);
    gs.setInputCloud(cloud);
    gs.filter(non_ground);
    gs.getGround(ground);

    PCL_WARN("Points before: %d\n", cloud->size());
    PCL_WARN("Points after : %d\n", non_ground.size());
    PCL_WARN("Points removed : %d\n", ground.size()); 
    PCL_WARN("-----------------------------------\n");

    // Back to sensor_msgs
    //pcl::toROSMsg(cloud_filtered, output);
    pcl::toROSMsg(non_ground, out_non_ground);
    pcl::toROSMsg(ground, out_ground);

    // Publish the data.
    pub_non_ground.publish (out_non_ground);
    pub_ground.publish (out_ground);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "Ground_Segmentation");
    ros::NodeHandle nh;

    //std::string topic = nh.resolveName("/autonomoose/velo/pointcloud");
    std::string topic = nh.resolveName("/kitti/velo/pointcloud");
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe (topic, 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2> ("Non_Ground", 1);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2> ("Ground", 1);

    // Spin
    ros::spin ();
}