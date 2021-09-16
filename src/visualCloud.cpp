#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
// #include <experimental/filesystem>
#include <unistd.h>
#include <stdio.h>
#include <ros/package.h>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

// using namespace std::experimental::filesystem::v1;


int
main (int argc, char **argv)
{
    std::string path = ros::package::getPath("msca3dp") + "/pcd/";
    std::string filename = path + "model.pcd";
    CloudPtr model (new Cloud);
    // pcl::io::loadPCDFile<pcl::PointXYZ>("scan.pcd", *model);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *model) == -1) 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        std::cout << "no file" << '\n';
        return (-1);
    }
    std::cout << "Loaded " << model->size () << " data points from model.pcd" << std::endl;

    CloudPtr scan0 (new Cloud);
    // pcl::io::loadPCDFile<pcl::PointXYZ>("scan.pcd", *model);
    // filename = path + "scan0.pcd";
    std::string filename2 = path +  argv[1];
   
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename2, *scan0) == -1) 
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        std::cout << "no file" << '\n';
        return (-1);
    }
    std::cout << "Loaded " << scan0->size () << " data points from scan.pcd" << std::endl;
    // Initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr
    viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    // viewer->addPointCloud(model,"model");
    viewer->addPointCloud(scan0,"model");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model");
    // // Coloring and visualizing target cloud (red).
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    // target_color (scan0, 255, 0, 0);
    // viewer->addPointCloud<pcl::PointXYZ> (scan0, target_color, "target cloud");
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //                                                 10, "target cloud");



    // Starting visualizer
    viewer->addCoordinateSystem (0.1, "global");
    viewer->initCameraParameters ();


    // Wait until visualizer window is closed.
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        // std::this_thread::sleep_for(100ms);
        // boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  

  return (0);
}