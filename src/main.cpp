#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

#define grid_size 100

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create data.
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> cloud_filtered;

    pcl::IndicesPtr inliers (new pcl::Indices()), tmp_inliers (new pcl::Indices()) ;

    Eigen::Vector4f max_pt, min_pt;

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    //Convert Point Cloud 
    pcl::fromROSMsg(*input, *cloud);

    //Filter
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    const float max_y = max_pt.y(), max_x = max_pt.y(), min_y = min_pt.y(), min_x = min_pt.x();

    for(float j = max_y; j > min_y ; j -= grid_size)
    {
        max_pt.y() = j;
        min_pt.y() = max_pt.y() - grid_size;

        for(float i = min_x; i < max_x ; i += grid_size)
        {
            min_pt.x() = i;
            max_pt.x() = min_pt.x() + grid_size;
            
            pcl::getPointsInBox(*cloud, min_pt, max_pt, *tmp_inliers);
            inliers->insert( inliers->end(), tmp_inliers->begin(), tmp_inliers->end() );
        }
    } 
       
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices(inliers);
    extract.setNegative (false);
    extract.filter(cloud_filtered);

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