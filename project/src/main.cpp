#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// from website
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

// from reys website
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// from harts website
#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>

// from reys website pt2
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

//tf 
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <utility>
#include <tf2_ros/transform_broadcaster.h>

ros::Publisher table_pub;
ros::Publisher not_table_pub;
ros::Publisher cyl_pub;
ros::Publisher cyl_pub2;

ros::Publisher pub2;
typedef pcl::PointXYZRGB PointT;


//return two point clouds
std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
    cloud_segment(const sensor_msgs::PointCloud2& input) {
    
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(input, cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud,*temp_cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    // Optional
    seg.setOptimizeCoefficients (true);
    
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (temp_cloud);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr not_table_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    extract.setInputCloud (temp_cloud);
    extract.setIndices (inliers);
    extract.setNegative (false); 
    extract.filter (*table_cloud);

    extract.setNegative (true); 
    extract.filter (*not_table_cloud);

  return std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr> (table_cloud, not_table_cloud);
}


//cylinder segmentation method
void cloud_cb3 ( sensor_msgs::PointCloud2& input)
{
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // We added this
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(input, cloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud2,*cloud);

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0, 0.05);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);

   //publish tf coeff
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/head_rgbd_sensor_rgb_frame";
    transformStamped.child_frame_id = "/cylinder";


    transformStamped.transform.translation.x = coefficients_cylinder->values[0];
    transformStamped.transform.translation.y = coefficients_cylinder->values[1];
    transformStamped.transform.translation.z = coefficients_cylinder->values[2];

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    static 	tf::TransformBroadcaster tf_br;
  	tf_br.sendTransform( transformStamped);

    // We also added this
    pcl::PCLPointCloud2 outputInProgress;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloud_cylinder, outputInProgress);
    pcl_conversions::fromPCL(outputInProgress, output);
    pub2.publish (output);
}


//sphere segmentation method
void cloud_cb4 ( sensor_msgs::PointCloud2& input)
{
    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
    // pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // We added this
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(input, cloud2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(cloud2,*cloud);

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_sphere (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_sphere (new pcl::PointIndices);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 1.5);
    pass.filter (*cloud_filtered);
    
    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for sphere segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations(1000000);
    seg.setDistanceThreshold (0.01);
    seg.setRadiusLimits (0, 0.15);
    seg.setInputCloud (cloud_filtered2);
    seg.setEpsAngle(15 / (180/3.141592654));
    seg.setInputNormals (cloud_normals2);


    // Obtain the sphere inliers and coefficients
    seg.segment (*inliers_sphere, *coefficients_sphere);

    // Write the sphere inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_sphere);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_sphere (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_sphere);

    //publish tf coeff
    geometry_msgs::TransformStamped transformStamped;
  
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "/head_rgbd_sensor_rgb_frame";
    transformStamped.child_frame_id = "/sphere";


    transformStamped.transform.translation.x = coefficients_sphere->values[0];
    transformStamped.transform.translation.y = coefficients_sphere->values[1];
    transformStamped.transform.translation.z = coefficients_sphere->values[2];

    transformStamped.transform.rotation.x = 0;
    transformStamped.transform.rotation.y = 0;
    transformStamped.transform.rotation.z = 0;
    transformStamped.transform.rotation.w = 1;

    static 	tf::TransformBroadcaster tf_br;
  	tf_br.sendTransform( transformStamped);

    // We also added this
    pcl::PCLPointCloud2 outputInProgress;
    sensor_msgs::PointCloud2 output;
    pcl::toPCLPointCloud2(*cloud_sphere, outputInProgress);
    pcl_conversions::fromPCL(outputInProgress, output);
    cyl_pub2.publish (output);
}


//segment out table and everything but the table
void cloud_cb (const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PCLPointCloud2 cloud2;
    pcl_conversions::toPCL(*input, cloud2);
    sensor_msgs::PointCloud2 will_become_cylinder;
    pcl_conversions::fromPCL(cloud2, will_become_cylinder);

    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr> table_clouds = cloud_segment(will_become_cylinder);

    // Create a container for the data.
    pcl::PCLPointCloud2 outputInProgressA;
    sensor_msgs::PointCloud2 outputA;
    
    pcl::toPCLPointCloud2(*table_clouds.first, outputInProgressA);
    pcl_conversions::fromPCL(outputInProgressA, outputA);
  
    pcl::PCLPointCloud2 outputInProgressB;

    sensor_msgs::PointCloud2 outputBetter;
    
    pcl::toPCLPointCloud2(*table_clouds.second, outputInProgressB);
    pcl_conversions::fromPCL(outputInProgressB, outputBetter);

   //Do data processing here...
    ROS_INFO("Now publishing table clouds");


    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr> object_clouds= cloud_segment(outputBetter);

    pcl::PCLPointCloud2 outputInProgressA2;
    sensor_msgs::PointCloud2 outputA2;
    
    pcl::toPCLPointCloud2(*object_clouds.second, outputInProgressA2);
    pcl_conversions::fromPCL(outputInProgressA2, outputA2);

    cyl_pub.publish (outputA2);

    cloud_cb3(will_become_cylinder); 
    cloud_cb4(will_become_cylinder);

    // Publish the data.
    table_pub.publish (outputA);
    not_table_pub.publish (outputBetter); 
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  table_pub = nh.advertise<sensor_msgs::PointCloud2> ("table", 1);
  not_table_pub = nh.advertise<sensor_msgs::PointCloud2> ("not_table", 1);
  cyl_pub = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("please_cylinder", 1);
  cyl_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("please_sphere", 1);

  ros::spin ();
}