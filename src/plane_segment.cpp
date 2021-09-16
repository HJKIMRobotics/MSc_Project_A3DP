#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/features/normal_3d.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

class PlaneSegment
{
public:

    PlaneSegment();
    
    PlaneSegment(std::string path)
    : est_based(false)
    {
        // ROS node generation
        ros::NodeHandle nh;
        
        theta = M_PI/2;
        nh.param("/plane_segment/est_based",est_based,est_based);

        // Initialize subscriber to the raw point cloud
        ros::Subscriber sub = nh.subscribe (path, 1,
                                            &PlaneSegment::pointsCB, this);                               
        
        _planePub = nh.advertise<sensor_msgs::PointCloud2 >("inliers",2);

        ros::spin ();
    };

    void getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    pcl::PointIndices::Ptr inliers_plane)
    {
        // Create SAC segmentor and setup planar model
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);

        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud);

        // Create segmentation object for planar model and set all the params
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment (*inliers_plane, *coefficients);

        // Extract planar inliers 
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers_plane);

        // Remove outliers
        extract.setNegative (true);
        extract.filter (*cloud);
        
    }

    void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        // min and max values in z axis to keep
        pass.setFilterLimits (0.0, 0.4); 
        pass.filter (*cloud);

    }

    void pointsCB(const sensor_msgs::PointCloud2ConstPtr& input)
    {
        // Conver from sensor_msg to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg (*input, *cloud);

        // Filter
        filter(cloud);

        // Segmentation
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
        getPlane(cloud, inliers_plane);

        if (cloud->points.empty())
        {
            ROS_ERROR_STREAM_NAMED("plane_segment", "Can't find the plane");
            return;
        }


        // Publish the inlier plane
        sensor_msgs::PointCloud2 planeCloud;
        
        if(est_based)
        {
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
            transform.rotate (Eigen::AngleAxisf (-theta, Eigen::Vector3f::UnitZ()));
            pcl::PointCloud<pcl::PointXYZ>::Ptr transC (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud (*cloud, *transC, transform);

            pcl::toROSMsg(*transC, planeCloud);
            planeCloud.header = input->header;
            planeCloud.header.frame_id = "estimated_link";
        }
        else
        {
            pcl::toROSMsg(*cloud, planeCloud);
            planeCloud.header = input->header;
            planeCloud.header.frame_id = "camera_link";
        }
        
        _planePub.publish(planeCloud);

    }

private:

    ros::Publisher _planePub;
    float theta;
    bool est_based;
};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "plane_segment");

    PlaneSegment ps(argv[1]);
    
}

