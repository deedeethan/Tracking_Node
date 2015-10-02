/*

Includes helper functions to compute the centroid of a given cloud,
to run kdtree search on several different orientations of the same
input point cloud, and to manually rotate an input point cloud
by a given number of degrees and translate it by a given distance.

Used for testing.

*/

#include <iostream>
#include <cmath>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <climits>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// This function displays the help
void showHelp(char * program_name)
{
  std::cout << std::endl;
  std::cout << "Usage: " << program_name << " cloud_filename.pcd" << std::endl;
  std::cout << "-h:  Show this help." << std::endl;
}


// Computes the centroid of a given cloud
Eigen::Vector4f compute_centroid(pcl::PointCloud<pcl::PointXYZ> cloud)
{
  Eigen::Vector4f centroid(4);
  centroid.setZero ();
  // For each point in the cloud
  int cp = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      centroid += cloud.points[i].getVector4fMap ();
    centroid[3] = 0;
    centroid /= cloud.points.size ();
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) ||
          !pcl_isfinite (cloud.points[i].y) ||
          !pcl_isfinite (cloud.points[i].z))
        continue;

      centroid += cloud.points[i].getVector4fMap ();
      cp++;
    }
    centroid[3] = 0;
    centroid /= cp;
  }

  return centroid;
}


// I need to change the parameters to account for the
// transformation matrices. I need to save the best TM
// so I can combine it with the icp TM.
//
// Does kdtree search on each rotation of 45 degrees around
// the central point of the moved_cloud
Eigen::Matrix4f kdtree_search(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr moved_cloud, float radius)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // Total number of nearest neighbors
  int num_matches = 0;
  // Return the cloud with the most number of nearest neighbors
  // within a given radius of the searchPoint
  int max_neighbors = 0;
  pcl::PointCloud<pcl::PointXYZ>::Ptr best_fit_cloud ;
  Eigen::Matrix4f best_fit_transform = Eigen::Matrix4f::Identity();

  Eigen::Vector4f centroid1 = compute_centroid(*moved_cloud);
  cout << "centroid of source cloud - " << centroid1(0)
  << ", " << centroid1(1) << ", " << centroid1(2) << endl;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

  for (int i = 0; i < 8; i++)
  {
    float theta = 45 * i + 45;
    transform_1 (0,0) = cos (theta);
    transform_1 (0,2) = -sin (theta);
    transform_1 (2,0) = sin (theta);
    transform_1 (2,2) = cos (theta);
    cout << "cloud with " << theta << " degrees of rotation" << endl;
pcl::transformPointCloud (*moved_cloud, *rotated_cloud, transform_1);

// Probably need to compute centroid of the new transformed cloud
// because the transformation seems to translate it
  Eigen::Vector4f centroid2 = compute_centroid(*rotated_cloud);
  cout << "centroid of rotated cloud - " << centroid2(0)
  << ", " << centroid2(1) << ", " << centroid2(2) << endl;
  float distance_x = centroid1(0) - centroid2(0);
  float distance_y = centroid1(1) - centroid2(1);
  float distance_z = centroid1(2) - centroid2(2);
  cout << "distance between centroids: (" << distance_x << ", " << distance_y << ", " << distance_z << ")" << endl;
  transform_2 (0,3) = (distance_x);
  transform_2 (1,3) = (distance_y);
  transform_2 (2,3) = (distance_z);
pcl::transformPointCloud (*rotated_cloud, *transformed_cloud, transform_2);


    // Rotate the cloud by 45 degrees each time
    // May want to add some random translation as well.
    // This would correspond to doing kdtree search on a number of
    // clouds that are presumably close to the target point cloud.
    kdtree.setInputCloud(transformed_cloud);

    // Run the kdtree search on every 10th point of the moved_cloud
    // Increase this number to speed up the search
    // Test with different increments of i to see effect on speed
    for (int j = 0; j < (*moved_cloud).points.size(); j += 10)
    {
      pcl::PointXYZ searchPoint = moved_cloud->points[j];
      int num_neighbors = kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      num_matches += num_neighbors;
      cout << "performed kdtree nearest neighbor search, found " << num_neighbors << " within " << radius << " radius" << endl;
    }
    cout << "num_matches = " << num_matches << endl;
    cout << "max_neighbors = " << max_neighbors << endl;

    if (num_matches > max_neighbors) {
      max_neighbors = num_matches;
      best_fit_cloud = transformed_cloud;
      // are these transforms relative? or absolute?
      // this currently calculates relative
      // should be transform_2 * transform_1 if absolute
      best_fit_transform = transform_2 * transform_1;
     }
    num_matches = 0;
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n"
      "                        blue = moved point cloud\n"
      "                        green = rotated point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);  // white
  // We add the point cloud to the viewer and pass the color handler
//  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_cloud_color_handler (rotated_cloud, 20, 245, 20); // green
  viewer.addPointCloud (rotated_cloud, rotated_cloud_color_handler, "rotated_cloud");


  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moved_cloud_color_handler (moved_cloud, 20, 230, 230); // blue
  viewer.addPointCloud (moved_cloud, moved_cloud_color_handler, "moved_cloud");

  viewer.addCoordinateSystem (1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "moved_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  }
  // check whether the translations are relative or absolute
  pcl::PointCloud<pcl::PointXYZ>::Ptr best_transform_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::transformPointCloud (*moved_cloud, *best_transform_cloud, best_fit_transform);

  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_transform_cloud_color_handler (best_transform_cloud, 245, 20, 20); // red
  viewer.addPointCloud (best_transform_cloud, best_transform_cloud_color_handler, "best_transform_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_fit_cloud_color_handler (best_fit_cloud, 20, 245, 20); // green
  viewer.addPointCloud (best_transform_cloud, best_fit_cloud_color_handler, "best_fit_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> input_cloud_color_handler (input_cloud, 255, 255, 255); // white
  viewer.addPointCloud (input_cloud, input_cloud_color_handler, "input_cloud");

  viewer.addCoordinateSystem (1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "best_transform_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "best_fit_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input_cloud");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  return best_fit_transform;
}


// Main function
int main (int argc, char** argv)
{

  // Show help
  if (pcl::console::find_switch (argc, argv, "-h") || pcl::console::find_switch (argc, argv, "--help")) {
    showHelp (argv[0]);
    return 0;
  }

  // Fetch point cloud filename in arguments | Works with PCD files
  std::vector<int> filenames;
  bool file_is_pcd = false;

  if (filenames.size () != 2)  {
    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

    if (filenames.size () != 2) {
      showHelp (argv[0]);
      return -1;
    } else {
      file_is_pcd = true;
    }
  }

  // Load file | Works with PCD and PLY files
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // The source cloud is the original cloud
    if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
      showHelp (argv[0]);
      return -1;
    }

  // The moved cloud is the original cloud after a transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr moved_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (argv[filenames[1]], *moved_cloud);



// compute the distance between the source_cloud's centroid and
// the moved_cloud's centroid
/*
  Eigen::Vector4f zero(4);
  zero << 0,0,0,0;
  unsigned int centroid1 = pcl::compute3DCentroid(*source_cloud, zero);
  unsigned int centroid2 = pcl::compute3DCentroid(*moved_cloud, zero);
  float distance = 0;
  float centroid1d = (float)centroid1;
  float centroid2d = (float)centroid2;
 // ints or floats won't do. I need a 3D vector or something
// so i can translate the x,y,z points over to the new location.
// unfortunately, the only other compute centroid function I can
// find returns void.
  distance = (centroid1d - centroid2d) / 1000.0;
  cout << centroid1 << std::endl;
  cout << centroid2 << std::endl;
  cout << distance << std::endl;
*/


/*
If I'm going to translate the new cloud to the correct
centroid position later, I don't need this initial translation.
  Eigen::Vector4f centroid2 = compute_centroid(*moved_cloud);
  float distance_x = centroid2(0) - centroid1(0);
  float distance_y = centroid2(1) - centroid1(1);
  float distance_z = centroid2(2) - centroid1(2);
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//  float theta = 45; // The angle of rotation in radians
//  transform_1 (2,0) = sin (theta);
//  transform_1 (2,1) = cos (theta);
  transform_1 (0,3) = abs(distance_x);
  transform_1 (1,3) = abs(distance_y);
  transform_1 (2,3) = abs(distance_z);
*/


  // Executing the transformation
//  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//  pcl::transformPointCloud (*source_cloud, *transform_cloud, transform_1);

//  transformed_cloud = kdtree_search(moved_cloud, transform_cloud);


/*
 for (int i = 0; i < 8; i++)
{
// this transformation isn't giving me an overlay of the moved
// cloud at different angle rotations
// it's accompanied by a translation that puts the resulting
// cloud too far from the moved_cloud, so the kdtree estimate
// won't be too good
// However, if icp can correct for this, it might be ok
// ideally, I would want this to work as well as possible
// to speed up icp.
// I also am not passing the original centroid-translated
// point cloud to this kdtree search. I need to find a way to do that
//
// The issue here is that I'm not translating the cloud correctly
// based on the distance between the centroids. It's more complicated
// than just translating the cloud by the distance.
// Look at the picture Robbie drew. There's lin alg involved.

// passing in 0 for theta doesn't work because cos(0) = 1

// Add 45 degrees so that you don't do cos 0
*/

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  Eigen::Matrix4f best_fit_transform = kdtree_search(source_cloud, moved_cloud, 0.04);
  pcl::transformPointCloud (*moved_cloud, *transformed_cloud, best_fit_transform);
  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n"
      "                        blue = moved point cloud\n");
//      "                        green = rotated point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);  // white
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");

/*
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_cloud_color_handler (rotated_cloud, 20, 245, 20); // green
  viewer.addPointCloud (rotated_cloud, rotated_cloud_color_handler, "rotated_cloud");
*/

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moved_cloud_color_handler (moved_cloud, 20, 230, 230); // blue
  viewer.addPointCloud (moved_cloud, moved_cloud_color_handler, "moved_cloud");

  viewer.addCoordinateSystem (1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "moved_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  return 0;
}
