This project is a ROS node that implements tracking for an arbitrary object.
The user needs
  - a Kinect
  - object to be tracked
  - a PCD file for the object to be tracked
  - computer with ROS installed

The file pointcloud_filtering.cpp is the main file that has the code
for the entire algorithm.

Run
    make
on the command line to compile it, then
    ./bin/pointcloud_filtering
while the Kinect and object are set up to run it.

The other files in this directory are
  - pcd file of primary object I used to test, big_triangle.pcd
  - test_pcd_files
      pcd files containing test point clouds with certain orientations
  - test_cpp_files
      cpp files containing implementations of pre-existing algorithms
    from the PCL documentation for testing (iterative_closest_point.cpp,
    kdtree_search.cpp, matrix_transform.cpp)
      cpp files containing data points from tests so I could graph them
    and see trends (filter_time.cpp, fitness_score.cpp, icp_time.cpp)
  - Data for filtering and icp.xlsx is an Excel file of graphs
    used for data analysis of icp time vs accuracy


