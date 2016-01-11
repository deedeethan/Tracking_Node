/*
 * filter_faster.cpp
 *
 * DeeDee Han
 * dthan@andrew.cmu.edu
 *
 * My attempt to write my own filtering function because I thought
 * the existing one I was using wasn't fast enough.
 * As far as I remember, doesn't compile.
 * Saved for reference.
 */

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
#include <pcl/common/concatenate.h>
#include <pcl/common/io.h>

typedef pcl::PointXYZ      Point;
typedef pcl::PointCloud<Point> PointCloud;

class PointCloudFiltering {

  // ROS properties
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber point_cloud_sub_;
  PointCloud::Ptr cloud_transformed;
  // Publisher to send out the filtered point cloud
  ros::Publisher point_cloud_filtered_;

  // Filter parameters
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  double z_filter_min_;
  double z_filter_max_;

  bool apply_xyz_limits_;

public:

  PointCloudFiltering() : nh_private_("~")
  {
    // Read the parameters from the parameter server (set defaults)
    nh_private_.param("apply_xyz_limits", apply_xyz_limits_, true);
    nh_private_.param("x_filter_min", x_filter_min_, -0.3);
    nh_private_.param("x_filter_max", x_filter_max_, 0.45);
    nh_private_.param("y_filter_min", y_filter_min_, -0.25);
    nh_private_.param("y_filter_max", y_filter_max_, 0.2);
    // the z filter min should not change because the Kinect can't detect
    // objects that are less than ~50 cm away
    nh_private_.param("z_filter_min", z_filter_min_, 0.5);
    // the max can change, however. The Kinect can sense objects
    // up to 3 m away
    nh_private_.param("z_filter_max", z_filter_max_, 0.9);

    point_cloud_sub_ = nh_.subscribe<PointCloud>(
      "camera/depth_registered/points",
      1, //queue size 1, want larger to store more messages?
      &PointCloudFiltering::
      pointCloudCb,
      this);

//    cloud_transformed =
    // Declare the point cloud filtered topic
    point_cloud_filtered_ = nh_private_.advertise<PointCloud>("output", 1);
  }

  void pointCloudCb(const PointCloud::ConstPtr& point_cloud)
  {
    PointCloud cloud = *point_cloud;
    PointCloud::Ptr input_cloud(&cloud);

    PointCloud::Ptr cloud_filtered_ptr(new PointCloud);
    cloud_filtered_ptr = filter(input_cloud);
/*    pcl::PassThrough<Point> pass;

    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_filter_min_, x_filter_max_);
    pass.setInputCloud(input_cloud);
    pass.filter(*cloud_filtered_ptr);
*/
    // PointCloud::Ptr cloud_downsampled = filter(cloud.makeShared());
    // ideally want to pass two point clouds of the same object to icp algorithm
  //  cloud_transformed = iterative_closest_point(cloud_downsampled, cloud_transformed);

    // Publish the filtered point cloud
    point_cloud_filtered_.publish(cloud_filtered_ptr);

    // Save filtered cloud to a pcd file
    pcl::io::savePCDFileASCII("filtered.pcd", *cloud_filtered_ptr);
  }

  PointCloud::Ptr filter(PointCloud::Ptr cloud)
  {
    PointCloud::Ptr cloud_out(new PointCloud);
    cloud_out = cloud;
    size_t oii = 0;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
      if (cloud->points[i].x < x_filter_min_ || cloud->points[i].x > x_filter_max_ || cloud->points[i].y < y_filter_min_ || cloud->points[i].y > y_filter_max_ || cloud->points[i].z < z_filter_min_ || cloud->points[i].z > z_filter_max_)
      {
        continue;
      }
      cloud_out->points[oii++] = cloud->points[i];
    }
    return cloud_out;
  }

/*
  // Filter the points, but don't save the points discarded
   void pcl::PassThrough<Point>::filter (PointCloud::Ptr cloud)
  {
    // output indices iterator
    int oii = 0;

    if (apply_xyz_limits_)
    {
      oud->points[i].x < x_filter_min_ || cloud->points[i].x > x_filt    er_max_/ Attempt to get the field name's index
      std::vector<pcl::PCLPointField> fields;
      int distance_idx = pcl::getFieldIndex(*input_, filter_field_name_, fields);

      // Filter for non-finite entries and the specified field limits
      // iii is input indices iterator
      for (int iii = 0; iii < static_cast<int> (indices_->size()); ++iii)
      {
        // non-finite entries
        if (!pcl_isfinite (input_->points[(*indices_)[iii]].x) ||
            !pcl_isfinite (input_->points[(*indices_)[iii]].y) ||
            !pcl_isfinite (input_->points[(*indices_)[iii]].z))
        {
 	  continue;
        }

	// Get the field's value
 	const uint8_t* pt_data = reinterpret_cast<const uint8_t*> (&input_->points[(*indices_)[iii]]);
  	float field_value = 0;
        memcpy (&field_value, pt_data + fields[distance_idx].offset, sizeof(float));

  	// Remove NAN/INF/-INF values
        if (!pcl_isfinite(field_value))
 	{
	  continue;
	}

	// Outside of field limits
	if (!negative_ && (field_value < filter_limit_min_ || field_value > filter_limit_max))
	{
	  continue;
	}

	// otherwise it was an inlier and stays in output cloud
	cloud[oii++] = (*indices_)[iii];
      }
    }

    cloud.resize(oii);
  }
*/
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_faster");
  PointCloudFiltering node;
  ros::spin();
  return 0;
}
