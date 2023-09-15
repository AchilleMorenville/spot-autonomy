# Header
std_msgs/Header header 

# Pose
geometry_msgs/Transform initial_guess

# Point cloud messages
sensor_msgs/PointCloud2 cloud  	    # original cloud deskewed
sensor_msgs/PointCloud2 cloud_edge  # extracted edge features
sensor_msgs/PointCloud2 cloud_flat  # extracted flat features