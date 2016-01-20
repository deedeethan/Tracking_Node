/*
 * This file is ode from the PCL documentation tutorial on KDTrees and
 * nearest neighbor search.
 *
 * The first example finds a specified number of nearest neighbors to
 * a specified searchPoint.
 *
 * The second example finds all the neighbors within a specified radius
 * of the searchPoint.
 *
 * Used for testing.
 */

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <vector>
#include <ctime>
#include <climits>

int main (int argc, char** argv)
{
  srand (time (NULL));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // Generate pointcloud data
  cloud->width = 1000;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }


  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  // Set input cloud
  kdtree.setInputCloud (cloud);

  // Set point of which to find nearest neighbors
  pcl::PointXYZ searchPoint;

  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

  // K nearest neighbor search

  // How many neighbors to search for
  // Finds this many neighbors of the searchPoint
  int K = 10;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  std::cout << "K nearest neighbor search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with K = " << K << std::endl;

  // Identify the nearest point to the searchPoint,
  // i.e. the point with the smallest squared distance to the searchPoint
  pcl::PointXYZ closest_point;
  float closest_point_distance = INT_MAX;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      if (pointNKNSquaredDistance[i] < closest_point_distance) {
        closest_point = cloud->points[ pointIdxNKNSearch[i] ];
	closest_point_distance = pointNKNSquaredDistance[i];
      }
    }
  }
  // Print the nearest neighbor's position and distance from searchPoint
  std::cout << "Nearest neighbor to given searchPoint: "
	    << "(" << closest_point.x
	    << ", " << closest_point.y
	    << ", " << closest_point.z
	    << ") " << std::endl
	    <<  "squaredDistance = " << closest_point_distance
	    << std::endl;


  // Neighbors within radius search

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  // Set the radius to 1 mm or maybe a litle larger
  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  // Finds all the neighbors within a given radius around the searchPoint
  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }


  return 0;
}
