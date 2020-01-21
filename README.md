# LiDAR-obstacle-detection
* This propose detection method, which is use only LiDAR data.
* This detects obstacles using quad-tree segmentation.
* Available for real-time self-driving systems.
* The maximum performance for LiDAR data is 30 Hz.

# Hardware
* HYUNDAI i30
* Ouster OS1 64 channel LiDAR
* Intel Core i5-8250U, 3.4Ghz
* 16G RAM
* Geforce 1050GTX

# Input topic
* '/points_raw'

# Output topic
* '/detected_boxes'
* '/obb_cluster'
* '/obb_boxes'

# Run 
roslaunch lidar_obstacle_detection lidar_detection.launch --screen 


# result
1. Segmentation boxes & obstacle detection boxes
<img src="LiDAR_obj_detect_pkg/images/Screenshot from 2020-01-20 16-06-20.png" width="100%" height="100%">

2. obstacle detection in PointCloud
<img src="LiDAR_obj_detect_pkg/images/Screenshot from 2020-01-20 16-06-51.png" width="100%" height="100%">

3. rqt graph
<img src="LiDAR_obj_detect_pkg/images/Screenshot from 2020-01-20 16-09-04.png" width="100%" height="100%">
