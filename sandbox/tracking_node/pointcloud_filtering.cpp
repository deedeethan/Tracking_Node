/*

This is all the main code for the tracking node.

The first section is declaring a class for the publisher, subscriber,
and callback. Defines the Point and PointCloud types for convenience.
Declares global variables and get and set functions
for filtering and callback.

The next section sets starting, default values for filtering parameters.
These parameters can be set from the command line when running this node.
(The default parameters right now work for the current testing setup
in lab. This consists of the Kinect propped up on a textbook, facing
the desktop. The big_triangle block is usually placed horizontally
on the wood board on top of the sticker.)
Declares the subscriber and publisher.
Publisher publishes messages of the type geometry_msgs::Pose, which
consists of a Point corresponding to an xyz position vector and
a quaternion corresponding to a 3x3 rotation matrix.

The third section is the main callback function for the node. This
function filters the incoming data for the object, transforms the
resulting cloud, and runs icp on the filtered cloud and an initial_guess
cloud.

*/

#include <iostream>
#include <math.h>
#include <ros/macros.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include </opt/ros/fuerte/include/ros/impl/time.h>
#include <std_msgs/String.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/registration_visualizer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include "tf/transform_datatypes.h"

//#include "crop_box.h"
//#include <dynamic_reconfigure/server.h>
//#include "cfg/dynam_reconfig.cfg"

typedef pcl::PointXYZ      Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Matrix4f Matrix;

class PointCloudFiltering {

public :
  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber point_cloud_sub_;
  // Publisher to send out the relevant transformation
  ros::Publisher tracking_;

  // Filter parameters
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  double z_filter_min_;
  double z_filter_max_;
  double voxel_size_;
  int mean_k_;
  double std_dev_thresh_;
  bool apply_xyz_limits_;
  bool apply_voxel_grid_;
  bool apply_outlier_removal_;

/*
These are the paramters for icp. This is if I want them to be
modified from the command line.
  int max_it;
  int r_max_it;
  double max_corres_dist;
  double transf_epsilon;
  double r_outlier_thres;
*/

  // for manual transforms and bounding box
  // Note: I didn't get the bounding box idea to work,
  // nor do I really need it. Just here for reference.
  double theta_;
//  Eigen::Vector4d min_pt_;
//  Eigen::Vector4d max_pt_;

  // Global variables
  PointCloud::Ptr obj_model;
  PointCloud::Ptr initial_guess;
  PointCloud::Ptr cloud_filtered;
  PointCloud::Ptr icp_cloud;
  Matrix initial_transform;
  Matrix icp_transform;
  bool first_it;
  int num_its;

// Get and set functions for global variables
public:

  void set_obj_model(PointCloud::Ptr cloud)
  {
    this->obj_model = cloud;
  }

  PointCloud::Ptr get_obj_model()
  {
    return this->obj_model;
  }

  void set_cloud_filtered(PointCloud::Ptr cloud)
  {
    this->cloud_filtered = cloud;
  }

  PointCloud::Ptr get_cloud_filtered()
  {
    return this->cloud_filtered;
  }

  void set_initial_guess(PointCloud::Ptr cloud)
  {
    this->initial_guess = cloud;
  }

  PointCloud::Ptr get_initial_guess()
  {
    return this->initial_guess;
  }

  void set_initial_transform(Matrix transform)
  {
    this->initial_transform = transform;
  }

  Matrix get_initial_transform()
  {
    return this->initial_transform;
  }

  void set_icp_cloud(PointCloud::Ptr cloud)
  {
    this->icp_cloud = cloud;
  }

  PointCloud::Ptr get_icp_cloud()
  {
    return this->icp_cloud;
  }

  void set_icp_transform(Matrix transform)
  {
    this->icp_transform = transform;
  }

  Matrix get_icp_transform()
  {
    return this->icp_transform;
  }

  // Filtering parameters
  PointCloudFiltering() : nh_private_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_private_.param("first_iteration", first_it, true);
    nh_private_.param("num_iterations", num_its, 0);
    nh_private_.param("apply_xyz_limits", apply_xyz_limits_, true);
    nh_private_.param("apply_voxel_grid", apply_voxel_grid_, true);
    nh_private_.param("apply_outlier_removal",
		      apply_outlier_removal_, true);
    nh_private_.param("x_filter_min", x_filter_min_, -0.2);
    nh_private_.param("x_filter_max", x_filter_max_, 0.45);
    nh_private_.param("y_filter_min", y_filter_min_, -0.20);
    nh_private_.param("y_filter_max", y_filter_max_, 0.2);
    // The z filter min should not change because the Kinect can't detect
    // objects that are less than ~50 cm away
    nh_private_.param("z_filter_min", z_filter_min_, 0.5);
    // The max can change, however. The Kinect can sense objects
    // up to 3 m away
    nh_private_.param("z_filter_max", z_filter_max_, 0.9);
    nh_private_.param("voxel_size", voxel_size_, 0.0099);
    nh_private_.param("mean_k", mean_k_, 50);
    nh_private_.param("std_dev_thresh", std_dev_thresh_, 1.0);

    // Angle value used for transforming the point cloud manually
    nh_private_.param("theta", theta_, -M_PI);

/*
icp parameters

    nh_private_.param("max_it", max_it, 200);
    nh_private_.param("RANSAC_it", r_max_it, 200);
    nh_private_.param("max_corres_dist", max_corres_dist, 0.01);
    nh_private_.param("transform_epsilon", transf_epsilon, 0.0001);
    nh_private_.param("outlier_threshold", r_outlier_thres, 0.05);
*/

/*
Parameters for a bounding box. Not currently in use.
Apparently, you can't set these parameters on the param server
because there is no predefined function that lets you do so

    nh_private_.param("box_x_min", min_pt_[0], -0.1);
    nh_private_.param("box_x_max", max_pt_[0], 0.1);
    nh_private_.param("box_y_min", min_pt_[1], -0.1);
    nh_private_.param("box_y_max", max_pt_[1], 0.1);
    nh_private_.param("box_z_min", min_pt_[2], 0.5);
    nh_private_.param("box_z_max", max_pt_[2], 0.6);
    nh_private_.param("box_id_min", min_pt_[3], 1);
    nh_private_.param("box_id_max", max_pt_[3], 1);
*/

    // Subscription to the point cloud result from stereo_image_proc
    point_cloud_sub_ = nh_.subscribe<PointCloud>(
      "camera/depth/points",
      1,
      &PointCloudFiltering::
      point_cloud_cb,
      this);

    // Declare the tracking topic
    tracking_ = nh_private_.advertise<geometry_msgs::Pose>("tracking", 10);
  }

  /* Callback function for incoming point cloud data from the Kinect

     First, filters the incoming data to isolate the object.
     Downsamples the filtered cloud to decrease the number of points
     icp must search through. Removes statistical outliers.

     On the first iteration of the function, the object model is
     used as an initial_guess for icp. Icp compares the object_model
     with the filtered cloud and outputs its best guess
     for a transformation. Usually takes 3-5 iterations for icp
     to output a correct match for the initial position.

     On each successive iteration, run icp with strict or loose
     parameters based on the transformation matrix of the previous
     iteration. If icp failed to find a match and returned the
     identity matrix, then relax the icp parameters for one iteration.
     (This works for large translations with no rotation).
     Otherwise, run icp with regular strict parameters.
     Right now, if a transformation involves both a rotation and a
     translation, icp cannot find a good match even with loose parameters.
     (Ideally, in this case, this function would then do a nearest neighbor
     search with kdtree and get an initial_guess with a closer
     orientation and xyz position to feed into icp.
     This would take a bit more time, but would make the algorithm
     more robust. Need to do more data analysis after I finish
     writing the code to see how much more time it takes.
     Right now, still working on writing it.
     Refer to the matrix_transform.cpp file for code).

     Refer to the file on Google Drive for data analysis of the
     current code. Includes several different transformations
     with corresponding filter time, icp time, and fitness scores.

     Display the relevant clouds using the PCLVisualizer.
     Convert the icp_transform to the geometry_msgs::Pose type,
     which consists of a Point for the xyz position
     and a Quaternion for the 3x3 rotation matrix.
     Publish the message to the topic.

  */
  void point_cloud_cb(const PointCloud::ConstPtr& point_cloud)
  {
    // Filter with downsampling and outlier removal
    // Time the filtering
    ros::Time begin = ros::Time::now();
    PointCloud cloud = *point_cloud;
    PointCloud::Ptr cloud_filtered = filter(cloud.makeShared());
    set_cloud_filtered(cloud_filtered);
    ros::Time next = ros::Time::now();
    // Save the filtered cloud to a pcd file for testing
    pcl::io::savePCDFileASCII("filtered.pcd", *cloud_filtered);

    if(first_it)
    {
      // Set obj_model cloud
      PointCloud::Ptr obj_model (new PointCloud);
      pcl::io::loadPCDFile ("big_triangle.pcd", *obj_model);
      set_obj_model(obj_model);

      // Rotate the filtered cloud 180 degrees to the right (clockwise)
      // to match orientation of obj_model cloud
      // Translate the filtered cloud to approximately the correct position
      Matrix initial_transform = Matrix::Identity();
      initial_transform (0,0) = cos (theta_);
      initial_transform (0,1) = -sin(theta_);
      initial_transform (1,0) = sin (theta_);
      initial_transform (1,1) = cos (theta_);
      initial_transform (0,3) = 0.1;
      initial_transform (1,3) = 0.15;
      initial_transform (2,3) = 0.62;

      PointCloud::Ptr initial_guess (new PointCloud);
      // Transfom the obj model to get an initial guess
      pcl::transformPointCloud(*obj_model, *initial_guess,
                               initial_transform);
      // Run icp and get the final transformation
      Matrix icp_transform =
             iterative_closest_point(initial_guess, cloud_filtered,
                                     0.1, 100, 0.0001, 0.05, 100);
      cout << "ICP transformation - " << endl;
      cout << icp_transform << endl;

      // Transform the initial_guess to get the icp_cloud
      // This is icp's guess of the transformation
      PointCloud::Ptr icp_cloud (new PointCloud);
      pcl::transformPointCloud(*initial_guess, *icp_cloud, icp_transform);

      set_initial_transform(initial_transform);
      set_initial_guess(initial_guess);
      set_icp_transform(icp_transform);
      set_icp_cloud(icp_cloud);

      ROS_INFO("First iteration, ready to begin tracking");
      first_it = false;
    }

    else
    {
/* if you get the message "start tracking" from the server,
   only run the else case because you've already found the
   object in the world. Send back the initial guess  */
      // Previous icp_transform becomes new initial_transform matrix
      // Previous icp_cloud becomes new initial_guess
      Matrix initial_transform = get_icp_transform();
      PointCloud::Ptr initial_guess = get_icp_cloud();

      // Check if previous icp_transform was the identity matrix.
      // If so, relax icp parameters because icp could not find a
      // good match (translation and/or rotation was too large).
      if (equal(Matrix::Identity(), icp_transform)) {
        Matrix kd_transform = compute_guess(icp_cloud,
                                            cloud_filtered, 0.03);
        PointCloud::Ptr kd_cloud (new PointCloud);
        pcl::transformPointCloud(*cloud_filtered, *kd_cloud,
                                 kd_transform);
        set_initial_guess(kd_cloud);
//        icp_transform = iterative_closest_point(initial_guess,
//                                                cloud_filtered,
//                                                0.01, 150, 0.0001, 0.005,
//                                                150) * kd_transform;

        ROS_INFO("icp run with kdtree_search");
      }

      // Run icp with original parameters and get the final transformation
  //    else {
        icp_transform = iterative_closest_point(initial_guess,
                                                cloud_filtered,
                                                0.01, 150, 0.0001, 0.005,
                                                150);
  //    }
      cout << "ICP transformation - " << endl;
      cout << icp_transform << endl << endl;

      PointCloud::Ptr icp_cloud (new PointCloud);
      pcl::transformPointCloud(*initial_guess, *icp_cloud, icp_transform);

      set_initial_guess(initial_guess);
      set_initial_transform(initial_transform);
      set_icp_transform(icp_transform);
      set_icp_cloud(icp_cloud);

      // Time the icp
      ros::Time final = ros::Time::now();
      ros::Duration filterTime = next - begin;
      ros::Duration icpTime = final - next;
      ROS_INFO("Filtering took: %f secs", (double) filterTime.toSec());
      ROS_INFO("ICP took: %f secs", (double) icpTime.toSec());
      saveInfo("filter_time.cpp", filterTime.toSec());
      saveInfo("icp_time.cpp", icpTime.toSec());

      geometry_msgs::Pose pose = convert_matrix_to_pose(icp_transform);

// Visualize every 10 iterations of icp
//    if (num_its % 10 == 0) {

/*  Testing the convertMatrixToQuat function

      Eigen::Matrix3d test_m = Eigen::Matrix3d::Identity();
      Eigen::Quaternionf quat = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
      test_m(0,0) = 0.999752;
      test_m(0,1) = 0.0145776;
      test_m(0,2) = -0.016892;
      test_m(1,0) = -0.0152866;
      test_m(1,1) = 0.998974;
      test_m(1,2) = -0.0426446;
      test_m(2,0) = 0.012533;
      test_m(2,1) = 0.0428917;
      test_m(2,2) = 0.998946;
      quat = convertMatrixToQuat(test_m);
*/

/*
       if (num_its == 5) {
         publish the message "ready to start tracking"
         and pause the Rosnode if possible. Start it up again
	 with the else case upon receiving the message start tracking
 	}
*/
      // Use the PCLVisualizer to view the obj_model, initial_guess,
      // filtered, and icp_transform clouds
      pcl::visualization::PCLVisualizer viewer
           ("ICP with obj model and inital guess");
      ROS_INFO("Display visualizer");

      // Define R,G,B colors for the point cloud
      // White is for the obj model cloud
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
           source_cloud_color_handler (obj_model, 255, 255, 255);
      // We add the point cloud to the viewer and pass the color handler
      viewer.addPointCloud (obj_model, source_cloud_color_handler,
                           "original_cloud");

      // Red is for the initial guess
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
           transformed_cloud_color_handler (initial_guess, 230, 20, 20);
      viewer.addPointCloud (initial_guess, transformed_cloud_color_handler,
                           "transformed_cloud");

      // Green is for the actual position of the transformed object
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
           cloud_out_color_handler (cloud_filtered, 20, 245, 20);
      viewer.addPointCloud (cloud_filtered, cloud_out_color_handler,
                           "cloud_out");

     // Blue is for the icp transform
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
           icp_color_handler (icp_cloud, 20, 20, 245);
      viewer.addPointCloud (icp_cloud, icp_color_handler, "icp_cloud");


      viewer.addCoordinateSystem (1.0, 0);
      // Setting background to a dark grey
      viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
      viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
         "original_cloud");
      viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
         "transformed_cloud");
      viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_out");
      viewer.setPointCloudRenderingProperties
        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_cloud");

      viewer.spinOnce (4500);
//    }

    // Increment the counter for the number of iterations
    // so I can visualize the clouds every 5 iterations of icp
    num_its++;

    // Publish the transform
    tracking_.publish(pose);

    /* Save the filtered point cloud to a pcd file
       Don't want to do this usually - too slow

    pcl::io::savePCDFileASCII("filtered.pcd", *cloud_filtered);
    pcl::io::savePCDFileASCII("initial_guess.pcd", *initial_guess);
    pcl::io::savePCDFileASCII("icp_transform.pcd", *icp_cloud);
    */
    }
  }

  // Helper function to filter a given point cloud
  // If the private parameter apply_xyz_limits is set to true,
  // then the given cloud is filtered based on the xyz_limits given.
  // If the parameter apply_voxel_grid is set to true,
  // then the cloud is downsampled with a given voxel_size.
  // If the parameter apply_outlier_removal is set to true,
  // then the cloud has statistical outliers removed.
  // Returns the filtered cloud.
  //
  // To increase the speed of icp, increase the voxel_size
  // so the filtered cloud has fewer points.
  PointCloud::Ptr filter(PointCloud::Ptr cloud)
  {
    // NAN and limit filtering
    PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
    pcl::PassThrough<Point> pass;
/*
    These values should be between the xyz min and max values
    for filtering points outside of object
    These bounds are for bounding boxes to remove the
    fingers of the robot's hand around the object, for example

    min_pt_[0] = 0.1;
    min_pt_[1] = 0.1;
    min_pt_[2] = 0.1;
    min_pt_[3] = 1;
    max_pt_[0] = 0.11;
    max_pt_[1] = 0.11;
    max_pt_[2] = 0.11;
    max_pt_[3] = 1;
*/

    if (apply_xyz_limits_)
    {
      // X-filtering
      pass.setFilterFieldName("x");
      pass.setFilterLimits(x_filter_min_, x_filter_max_);
      pass.setInputCloud(cloud);
      pass.filter(*cloud_filtered_ptr);
//      pass.setFilterLimits(min_pt_[0], max_pt_[0]);
//      pass.setInputCloud(cloud);
//      pass.filter(*cloud_filtered_ptr);

      // Y-filtering
      pass.setFilterFieldName("y");
      pass.setFilterLimits(y_filter_min_, y_filter_max_);
      pass.setInputCloud(cloud_filtered_ptr);
      pass.filter(*cloud_filtered_ptr);
//      pass.setFilterLimits(min_pt_[1], max_pt_[1]);
//      pass.setInputCloud(cloud);
//      pass.filter(*cloud_filtered_ptr);

      // Z-filtering
      pass.setFilterFieldName("z");
      pass.setFilterLimits(z_filter_min_, z_filter_max_);
      pass.setInputCloud(cloud_filtered_ptr);
      pass.filter(*cloud_filtered_ptr);
//     pass.setFilterLimits(min_pt_[2], max_pt_[2]);
//     pass.setInputCloud(cloud);
//     pass.filter(*cloud_filtered_ptr);
    }
    else
    {
      cloud_filtered_ptr = cloud;
    }


    // Downsampling using voxel grid
    PointCloud::Ptr cloud_downsampled_ptr(new PointCloud);

    if (apply_voxel_grid_)
    {
      pcl::VoxelGrid<Point> grid;
      grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      grid.setDownsampleAllData(true);
      grid.setInputCloud(cloud_filtered_ptr);
      grid.filter(*cloud_downsampled_ptr);
    }
    else
    {
      cloud_downsampled_ptr = cloud_filtered_ptr;
    }


    // Statistical outlier removal
    PointCloud::Ptr cloud_outlier_ptr(new PointCloud);

    if (apply_outlier_removal_)
    {
      pcl::StatisticalOutlierRemoval<Point> sor;
      sor.setInputCloud(cloud_downsampled_ptr);
      sor.setMeanK(mean_k_);
      sor.setStddevMulThresh(std_dev_thresh_);
      sor.filter(*cloud_outlier_ptr);
    }
    else
    {
      cloud_outlier_ptr = cloud_downsampled_ptr;
    }

    return cloud_outlier_ptr;
  }


  // Helper function to run icp on two input clouds
  // Parameters include:
  //   Max correspondence distance between two points
  //   Max number of iterations of icp
  //   Transformation epsilon
  //     (the difference between the previous transformation
  //      and the current estimated transformation is smaller
  //      than a user imposed value)
  //   RANSAC outlier threshold
  //   RANSAC max number of iterations
  //     (RANSAC is a randomized algorithm that tries to find
  //      the most number of matching points)
  //
  // Takes in two different point clouds and returns the transformation
  // to get from one to the other
  //
  // To speed up icp, decrease the number of iterations and
  // increase the transformation epsilon.
  Matrix iterative_closest_point(PointCloud::Ptr cloud_in,
                                 PointCloud::Ptr cloud_out,
                                 double max_corres_dist,
                                 int max_it,
                                 double transf_epsilon,
                                 double r_outlier_thres,
                                 int r_max_it)
  {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);

    // Set the max correspondence distance to 5cm
    // (e.g. correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (max_corres_dist);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (max_it);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (transf_epsilon);
    icp.setRANSACOutlierRejectionThreshold (r_outlier_thres);
    icp.setRANSACIterations (r_max_it);

// icp comes with a visualizer. However, I was unable
// to get it to work. Didn't compile for some reason.
/*    pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> regVis;
    regVis.setRegistration(icp);
    regVis.setMaximumDisplayedCorrespondences(100);
    regVis.startDisplay();
*/

    PointCloud::Ptr Final (new PointCloud);
    icp.align(*Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    saveInfo("fitness_score.cpp", icp.getFitnessScore());

  /*  regVis.stopDisplay();
    std::string help;
    std::cout << "Type any character and press enter to quit"
    << std::endl;
    std::cin >> help;
  */

    // Return the transformation
    return icp.getFinalTransformation();
  }

  // Returns true if two arrays are element-wise equal
  // Used to determine if one matrix is equivalent to another matrix,
  // in which case you relax the icp parameters because it does not
  // find enough correspondences
  bool equal (Matrix input, Matrix output)
  {
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        if (input(i,j) != output(i,j))
          return false;
      }
    }
    return true;
  }

  // Given a point cloud, compute_centroid will find the centroid
  // and return it as an Eigen::Vector4f
  Eigen::Vector4f compute_centroid(PointCloud cloud)
  {
    // Zero vector
    Eigen::Vector4f centroid(4);
    centroid.setZero();

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


  // Given icp's previous guess and the new filtered input cloud,
  // compute_guess finds the distance between the two centroids
  // of the clouds and does a kd-tree nearest neighbor search
  // to find the next initial_guess.
  // The new initial_guess is thus icp's previous guess
  // translated to a new position based on the distance between
  // the centroids, and rotated based on the orientation of the cloud
  // that has the most neighbors with the filtered input cloud.
  //
  // This is used when the object is rotated and translated
  // at the same time. Icp has difficulty finding a good match
  // after a transformation of this type, so we need to seed icp
  // with a better initial_guess.
  Matrix compute_guess(PointCloud::Ptr source_cloud,
                       PointCloud::Ptr moved_cloud, float radius)
  {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    // Total number of nearest neighbors for a given cloud
    int num_matches = 0;
    // Maximum number of nearest neighbors
    int max_neighbors = 0;

    // Return the transformation matrix to get to the cloud
    // with the most number of nearest neighbors
    // within a given radius of the searchPoint
    PointCloud::Ptr best_fit_cloud;
    Matrix best_fit_transform = Eigen::Matrix4f::Identity();

    // Compute the centroid of the moved_cloud
    Eigen::Vector4f centroid1 = compute_centroid(*moved_cloud);

    PointCloud::Ptr rotated_cloud (new PointCloud);
    PointCloud::Ptr transformed_cloud (new PointCloud);
    Matrix transform1 = Eigen::Matrix4f::Identity();
    Matrix transform2 = Eigen::Matrix4f::Identity();

    for (int i = 0; i < 8; i++)
    {
      float theta = 45 * i + 45;
      transform1 (0,0) = cos (theta);
      transform1 (0,2) = -sin (theta);
      transform1 (2,0) = sin (theta);
      transform1 (2,2) = cos (theta);
      cout << "cloud with " << theta << " degrees of rotation" << endl;
      pcl::transformPointCloud(*moved_cloud, *rotated_cloud, transform1);

      Eigen::Vector4f centroid2 = compute_centroid(*rotated_cloud);
      float distance_x = centroid1(0) - centroid2(0);
      float distance_y = centroid1(1) - centroid2(1);
      float distance_z = centroid1(2) - centroid2(2);
      transform2 (0,3) = (distance_x);
      transform2 (1,3) = (distance_y);
      transform2 (2,3) = (distance_z);
      pcl::transformPointCloud(*rotated_cloud, *transformed_cloud,
                                transform2);

      kdtree.setInputCloud(transformed_cloud);

      for (size_t j = 0; j < (*moved_cloud).points.size(); j += 10)
      {
        pcl::PointXYZ searchPoint = moved_cloud->points[j];
        int num_neighbors = kdtree.radiusSearch(searchPoint, radius,
                        pointIdxRadiusSearch, pointRadiusSquaredDistance);
        num_matches += num_neighbors;
        cout << "performed kdtree nearest neighbor search, found " <<
                num_neighbors << " within " << radius << " radius" << endl;
      }

      cout << "num_matches = " << num_matches << endl;
      cout << "max_neighbors = " << max_neighbors << endl;

      if (num_matches > max_neighbors) {
        max_neighbors = num_matches;
        best_fit_cloud = transformed_cloud;
        best_fit_transform = transform2 * transform1;
      }

      num_matches = 0;

      printf(  "\nPoint cloud colors :  white  = original point cloud\n"
               "                        red  = transformed point cloud\n"
               "                        blue = moved point cloud\n");
//               "                        green = rotated point cloud\n");
      pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rotated_cloud_color_handler (rotated_cloud, 20, 245, 20); // green
//          viewer.addPointCloud (rotated_cloud, rotated_cloud_color_handler,             "rotated_cloud");


      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
      viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moved_cloud_color_handler (moved_cloud, 20, 230, 230); // blue
      viewer.addPointCloud (moved_cloud, moved_cloud_color_handler, "moved_cloud");

      viewer.addCoordinateSystem (1.0, 0);
      viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
//      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "rotated_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "moved_cloud");

      while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr best_transform_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*moved_cloud, *best_transform_cloud,best_fit_transform);

    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_transform_cloud_color_handler (best_transform_cloud, 245, 20, 20); // red
    viewer.addPointCloud (best_transform_cloud,          best_transform_cloud_color_handler, "best_transform_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> best_fit_cloud_color_handler (best_fit_cloud, 20, 245, 20); // green
    viewer.addPointCloud (best_transform_cloud, best_fit_cloud_color_handler, "best_fit_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255); // white
    viewer.addPointCloud (source_cloud, source_cloud_color_handler, "source_cloud");

    viewer.addCoordinateSystem (1.0, 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a    dark grey

    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "best_transform_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "best_fit_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud");
    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce ();
    }

    return best_fit_transform;
  }


  // Saves the data from filtering time, icp time, and fitness score
  // into separate files
  // Used to graph data vs iteration number to look for trends
  void saveInfo(const std::string &file_name, double data)
  {
    std::ofstream fs;
    fs.open(file_name.c_str(), fstream::app | fstream::out);
    fs << data << std::endl;
    fs.close();
  }

  // Converts a 3x3 rotation matrix into a quaternion of the form
  // w + xi + yj + zk
  // Used to publish the position and orientation of the icp_transform
  // as a geometry_msgs::Pose object, which consists of a Point
  // and a Quaternion
  Eigen::Quaternionf convert_matrix_to_quat(Eigen::Matrix3d rotation)
  {
    // Identity quaternion
    Eigen::Quaternionf quat = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);

    // The elements of the rotation matrix
    double r11 = rotation(0,0);
    double r12 = rotation(0,1);
    double r13 = rotation(0,2);
    double r21 = rotation(1,0);
    double r22 = rotation(1,1);
    double r23 = rotation(1,2);
    double r31 = rotation(2,0);
    double r32 = rotation(2,1);
    double r33 = rotation(2,2);

    // Using the equations from Matt's book. There are 10 total
    double q0_sq = 0.25 * (1.0 + r11 + r22 + r33);
    double q1_sq = 0.25 * (1.0 + r11 - r22 - r33);
    double q2_sq = 0.25 * (1.0 - r11 + r22 - r33);
    double q3_sq = 0.25 * (1.0 - r11 - r22 + r33);

    // Values in the quaternion
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;

    // Find the max of the squared values
    double max1 = 0.0;
    double max2 = 0.0;
    double max3 = 0.0;
    max1 = std::max(q0_sq, q1_sq);
    max2 = std::max(max1, q2_sq);
    max3 = std::max(max2, q3_sq);

    // Solve for the other values using the other 6 equations
    if (max3 == q0_sq)
    {
      q0 = sqrt(q0_sq);
      q1 = (0.25 * (r32 - r23)) / q0;
      q2 = (0.25 * (r13 - r31)) / q0;
      q3 = (0.25 * (r21 - r12)) / q0;
    }
    else if (max3 == q1_sq)
    {
      q1 = sqrt(q1_sq);
      q0 = (0.25 * (r32 - r23)) / q1;
      q2 = (0.25 * (r12 + r21)) / q1;
      q3 = (0.25 * (r13 + r31)) / q1;
    }
    else if (max3 == q2_sq)
    {
      q2 = sqrt(q2_sq);
      q0 = (0.25 * (r13 - r31)) / q2;
      q1 = (0.25 * (r12 + r21)) / q2;
      q3 = (0.25 * (r23 + r32)) / q2;
    }
    else if (max3 == q3_sq)
    {
      q3 = sqrt(q3_sq);
      q0 = (0.25 * (r21 - r12)) / q3;
      q1 = (0.25 * (r13 + r31)) / q3;
      q2 = (0.25 * (r23 + r32)) / q3;
    }
    else
    {
      cout << "Double comparison is wrong, did not find match" << endl;
      return quat;
    }

    quat = Eigen::Quaternionf(q0, q1, q2, q3);

/* Print out the values of the quaternion. For testing

    cout << q0 << ", " << q1 << ", " << q2 << ", "
    << q3 << endl;
*/

    return quat;
  }


  // Convert the 4x4 transformation matrix to a quaternion
  // and a position vector, a point.
  // This is so the transformation can be published as a
  // geometry_msgs::Pose object, which consists of a Point
  // and a Quaternion that represents the 3x3 rotation matrix
  geometry_msgs::Pose convert_matrix_to_pose(Matrix icp_transform)
  {
    // Create a 3x3 rotation matrix
    Eigen::Matrix3d orient_matrix = Eigen::Matrix3d::Identity();
    for (int i = 0; i < 3; i++)
    {
    	for (int j = 0; j < 3; j++)
    	{
    	  orient_matrix(i,j) = icp_transform(i,j);
     }
    }

    // Create an identity quaternion and convert the 3x3 rotation
    // matrix to a quaternion
    Eigen::Quaternionf orient_quat = Eigen::Quaternionf(1,0,0,0);
    orient_quat = convert_matrix_to_quat(orient_matrix);

    // Convert Eigen::Quaternion to tf Quaternion
    tf::Quaternion tfQuat = tf::createIdentityQuaternion();
    tfQuat[0] = orient_quat.x();
    tfQuat[1] = orient_quat.y();
    tfQuat[2] = orient_quat.z();
    tfQuat[3] = orient_quat.w();

    // Create a geometry_msgs::Pose object and fill it with relevant info
    geometry_msgs::Pose pose;
    tf::Point pos = tf::Point(icp_transform(0,3), icp_transform(1,3),
                       				icp_transform(2,3));
    tf::pointTFToMsg(pos, pose.position);
    tf::quaternionTFToMsg(tfQuat, pose.orientation);

    return pose;
  }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_filtering");
  PointCloudFiltering node;

/* as long as there is no message that says stop, keep spinning */
  ros::spin();
  return 0;
}
