Project title: Machine-Material Feedback for Autonomous Adaptive 3D Printing
Author: Ka Kei Choi and Hyungjoo Kim
Supervisor and advisor: Prof. Simon Julier and PhD. Julis Sustarevas
Project description: 
In recent years, extensive research is being carried to examine how autonomous mobile 3D printing can be used in the additive manufacturing industry. Additive manufacturing is an efficient technology because it produces simple shapes and designs, and is a transformative manufacturing process approach. However, the quality of the printed layer produced largely depends on the precision of the 3D map reconstructed, which is used to provide feedback to a robot extruder controller. Therefore, the printing process is sensitive to the noise from the robot, sensors, and other components. This project proposes an online approach that uses local visual sensors to localise the sensor with AprilTag (a visual fiducial system) and detect printing deformations through a spatial change detection technique based on the occupancy map of a print. We further evaluated the proposed method with the RealSense D435i and L515 sensors in a Gazebo-simulated
and real world environment to determine the sensor that would be more suitable for a small-scale printing task. The performance of the printing task in the presence of different noises was also investigated through experiments.


ROS package require:
1. pcl_ros
2. apriltag_ros
3. octomap
4. octomap_server
5. realsense-ros
6. gazebo_ros_pkgs
7. usb_cam
8. xarm_ros (future works)

===========================================================================

Download the latest releases:
  https://github.com/JuliusSustarevas/_msc-a3dp/tree/anisa-branch

===========================================================================

Testing D435i model:

1st: roslaunch msca3dp view_d435i_model_rviz_gazebo.launch  # Plug-in the sensor model in Gazebo simulation
2nd: roslaunch msca3dp tag_detection_d435i.launch           # Launch tag detection (D435i) and camera calibration 
3th: roslaunch msca3dp simObj.launch			                  # Start spawning the cubics and follow the predetermined sensor trajectory
4th: roslaunch msca3dp octomap.launch		                    # Launch the plane segmentation and OctoMap mapping system
5th: roslaunch msca3dp change_detection.launch              # Launch change detection

Ctrl + c => Shutdown all processes
--------------------------------------------------------------------------

Testing L515 model:

1st: roslaunch msca3dp view_l515_model.launch               # Plug-in the sensor model in Gazebo simulation
2nd: roslaunch msca3dp tag_detection_l515.launch            # Launch tag detection (L515) and camera calibration 
3th: roslaunch msca3dp simObj.launch			                  # Start spawning the cubics and follow the predetermined sensor trajectory
4th: roslaunch msca3dp octomap.launch		                    # Launch the plane segmentation and OctoMap mapping system
5th: roslaunch msca3dp change_detection.launch              # Launch change detection

Ctrl + c => Shutdown all processes
-------------------------------------------------------------------------
The process for evaluating the results,

1st: Check ./ros file directory and there is your output files (i.e., output0.txt ...) if you want to check the occupancy probability during the tasks
2nd: Check ./ros file directory and there is your output files (i.e., traj.csv ...) if you want to check the predefined sensor trajectory (ground truth)
3rd: Go to "scripts" and evaluate the all occupancy probabiltiy for each experiment using output0~outputXXX.txt
4th: Go to MATLAB and import traj.csv file. You can make a 3D plot and check the trajectory.

============================================================================


Thank you so much and don't hesitate to send an email to us if you have questions.

Kind regards
Hyungjoo Kim & Ka Kei Choi
