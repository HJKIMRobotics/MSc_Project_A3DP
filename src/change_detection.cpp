#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <ros/package.h>
#include <stdio.h>

using PointType = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointType>;
using CloudConstPtr = Cloud::ConstPtr;
using CloudPtr = Cloud::Ptr;

class ChangeDetection
{
public:


    ChangeDetection();
    
    ChangeDetection(std::string path)
    : m_pcdfile("model.pcd"),
      m_enableprint(false),
      m_printfile("CD.csv")
    {
        // ROS node generation
        ros::NodeHandle nh;
        nh.param("/change_detection/pcdfilename",m_pcdfile,m_pcdfile);
        nh.param("/change_detection/enableprint",m_enableprint,m_enableprint);
        nh.param("/change_detection/printfile",m_printfile,m_printfile);

        std::string filepath = ros::package::getPath("msca3dp") + "/pcd/";
        std::string filename = filepath + m_pcdfile;
 
        _model = CloudPtr (new Cloud);
        if (pcl::io::loadPCDFile<PointType>(filename, *_model) == -1) 
        {
            PCL_ERROR ("Couldn't read file model.pcd \n");
            std::cout << "no file" << '\n';
        }
        modelsize = _model->size();

        if (remove(m_printfile.c_str()) != 0)
		    ROS_INFO("File deletion failed");
	    else
		    ROS_INFO("File deleted successfully");
	

        // Initialize subscriber to the raw point cloud
        ros::Subscriber sub = nh.subscribe (path, 1,
                                            &ChangeDetection::pointsCB, this);                               
        
        _changePub = nh.advertise<sensor_msgs::PointCloud2 >("changes",2);

        _resolution = 0.01f;

        ros::spin();

    };

    void printerr(int points)
    {
        std::ofstream out(m_printfile, std::ios_base::app);
        std::string line = std::to_string(ros::Time::now().toSec()) + ", "+ std::to_string(points) + "\n";
        // ROS_INFO_STREAM(line);
        out << line ;
        out.close();
    }


    void pointsCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // ROS_INFO("get scan");
        // Conver from sensor_msg to PCL point cloud
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        CloudPtr cloud (new Cloud);
        pcl::fromROSMsg (*input, *cloud);

        pcl::octree::OctreePointCloudChangeDetector<PointType> octree(_resolution);
        octree.setInputCloud (cloud);
        octree.addPointsFromInputCloud ();

        // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
        octree.switchBuffers ();

        // Add points from cloudB to octree
        octree.setInputCloud (_model);
        octree.addPointsFromInputCloud ();

        std::vector<int> newPointIdxVector;
        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels (newPointIdxVector);

        if(m_enableprint)
        {
            printerr(newPointIdxVector.size());
        }

        CloudPtr newCloud (new Cloud);
        pcl::copyPointCloud(*_model, newPointIdxVector,*newCloud);

        // Publish the inlier plane
        sensor_msgs::PointCloud2 changeC;
        pcl::toROSMsg(*newCloud, changeC);
        changeC.header = input->header;
        _changePub.publish(changeC);

    }

private:

    ros::Publisher _changePub;
    CloudPtr _model;
    
    int iter, fileidx, modelsize;
    float _resolution, ndt_res, filter_res;
    double step_size, trans_eps;
    std::string m_pcdfile, m_printfile;
    bool m_enableprint;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "change_detection");

    ChangeDetection cd(argv[1]);

}

