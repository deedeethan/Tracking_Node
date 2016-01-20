cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
#rosbuild_add_executable(talker src/talker.cpp)
#rosbuild_add_executable(listener src/listener.cpp)
	rosbuild_add_executable(pointcloud_filtering pointcloud_filtering.cpp)
#rosbuild_add_executable(pointcloud_viewer src/pointcloud_viewer.cpp)
#rosbuild_add_executable(crop_box src/crop_box.cpp)
#rosbuild_add_executable(filter src/filter_indices.h)
#rosbuild_add_executable(filter_faster filter_faster.cpp)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#project(pcd_write)
#project(pcd_read)
#project(pcl-matrix_transform)
#project(iterative_closest_point)
project(pcl_visualizer_viewports)
#project(kdtree_search)
#project(concatenate_clouds)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/vtk-5.8)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (pcd_write pcd_write.cpp)
#add_executable (pcd_read pcd_read.cpp)
#add_executable (matrix_transform matrix_transform.cpp)
#add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
#add_executable (iterative_closest_point iterative_closest_point.cpp)
#add_executable (kdtree_search kdtree_search.cpp)
#add_executable (concatenate_clouds concatenate_clouds.cpp)
#target_link_libraries (concatenate_clouds ${PCL_LIBRARIES})
#target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})
#target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})
#target_link_libraries (matrix_transform ${PCL_LIBRARIES})
	target_link_libraries(pointcloud_filtering ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so)
#target_link_libraries(kdtree_search ${PCL_LIBRARIES})
#target_link_libraries (pcd_write ${PCL_LIBRARIES})
#target_link_libraries (pcd_read ${PCL_LIBRARIES})
#target_link_libraries(${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so )

# Find VTK
find_package(VTK)
if (VTK_FOUND)
  include(${VTK_USE_FILE})
endif()
