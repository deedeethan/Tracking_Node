/*
 * iterative_closest_point.cpp
 *
 * DeeDee Han
 * dthan@andrew.cmu.edu
 *
 * Takes in a pcd file of data taken from the Kinect.
 * Filters the point cloud for the object.
 * Transforms the filtered cloud and runs icp.
 * Visualizes the input and resulting clouds.
 *
 * Used for testing.
 *
 */


#include <iostream>
#include <ros/macros.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
//#include <rostime_decl.h>
#include </opt/ros/fuerte/include/ros/impl/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZ      Point;
typedef pcl::PointCloud<Point> PointCloud;

int
 main (int argc, char** argv)
{
  // Get point cloud filename .pcd
  std::vector<int> filenames;
  bool file_is_pcd = false;

  if (filenames.size () != 1)  {
   filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
   file_is_pcd = true;
  }
// load the file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (argv[filenames[0]], *cloud_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[filenames[1]], *cloud_out);
// Filter the points
//  ros::Time begin = ros::Time::now();

  PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
  pcl::PassThrough<Point> pass(false);

  pass.setFilterFieldName("x");
  pass.setFilterLimits(-0.3, 0.45);
  pass.setKeepOrganized(false);
  pass.setInputCloud(cloud_out);
  pass.filter(*cloud_filtered_ptr);

  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.25, 0.2);
  pass.setKeepOrganized(false);
  pass.setInputCloud(cloud_filtered_ptr);
  pass.filter(*cloud_filtered_ptr);

  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.5, 0.9);
  pass.setKeepOrganized(false);
  pass.setInputCloud(cloud_filtered_ptr);
  pass.filter(*cloud_filtered_ptr);

//  ros::Duration time_filter = ros::Time::now() - begin;
//  double secs = time_filter.toSec();
//  std::cout << "time to filter: " << secs << endl;
/*
 Then downsample
//  begin = ros::Time::now();
  PointCloud::Ptr cloud_downsampled_ptr(new PointCloud);

  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(0.01, 0.01, 0.01);
  grid.setDownsampleAllData(true);
  grid.setInputCloud(cloud_filtered_ptr);
  grid.filter(*cloud_downsampled_ptr);
*/
//  ros::Duration time = ros::Time::now() - begin;
//  secs = time.toSec();
//  std::cout << "time to downsample: " << secs << endl;

/* Remove statistical outliers
  PointCloud::Ptr cloud_outlier_ptr(new PointCloud);

  pcl::StatisticalOutlierRemoval<Point> sor;
  sor.setInputCloud(cloud_downsampled_ptr);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*cloud_outlier_ptr);
*/
    float theta_ = -M_PI;
    Eigen::Matrix4f initial_transform = Eigen::Matrix4f::Identity();
    initial_transform (0,0) = cos (theta_);
    initial_transform (0,1) = -sin(theta_);
    initial_transform (1,0) = sin (theta_);
    initial_transform (1,1) = cos (theta_);
    initial_transform (0,3) = 0.1;
    initial_transform (1,3) = 0.12;
    initial_transform (2,3) = 0.62;

  PointCloud::Ptr cloud_transformed (new PointCloud);
  pcl::transformPointCloud(*cloud_in, *cloud_transformed, initial_transform);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_transformed);
  icp.setInputTarget(cloud_filtered_ptr);

  // May want to change these parameters
  // This is for accuracy. Loosen to get faster results
  // Set the max correspondence distance to 5cm (e.g. correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance (0.5);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (200);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (0.0001);
  icp.setRANSACOutlierRejectionThreshold (0.05);
  icp.setRANSACIterations (200);

//  begin = ros::Time::now();
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ> ());
  icp.align(*Final);
  std::cout << "ICP results - " << std::endl;
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  Eigen::Matrix4f transformation = icp.getFinalTransformation();
  std::cout << icp.getFinalTransformation() << std::endl;
  pcl::io::savePCDFileASCII("icp_transform.pcd", *Final);

//  ros::Duration time_icp = ros::Time::now() - begin;
//  secs = time_icp.toSec();
//  std::cout << "time to run icp: " << secs << endl;

  pcl::visualization::PCLVisualizer viewer ("Rigid transformation example");

   // Define R,G,B colors for the point cloud
// White is for the original filtered cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud_in, 255, 255, 255); // White
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud_in, source_cloud_color_handler, "original_cloud");


// Red is for the icp transform
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (Final, 230, 20, 20); // Red
  viewer.addPointCloud (Final, transformed_cloud_color_handler, "transformed_cloud");


// Green is for the actual position of the transformed object
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_out_color_handler (cloud_filtered_ptr, 20, 245, 20); // Green
  viewer.addPointCloud (cloud_filtered_ptr, cloud_out_color_handler, "cloud_out");

  // Blue is for the actual position of the transformed object
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_transformed_color_handler (cloud_transformed, 20, 20, 245); // Blue
  viewer.addPointCloud (cloud_transformed, cloud_transformed_color_handler, "initial_transform");

  viewer.addCoordinateSystem (1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }


 return (0);
}
