# Tracking_Node
Rosnode to track an object in 3D space in real-time
* Uses a Kinect to get a point cloud of the world
* Filters out points not corresponding to the object being tracked
* Prints out and publishes to the ROS server a transformation matrix for the transformation of the object from the previous frame to the current frame 
* Built at the Manipulation Lab of the Robotics Institute at Carnegie Mellon University from Oct 2014 - Dec 2015
