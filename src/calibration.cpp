#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

class Calibration
{
public:
    
    Calibration();
    
    Calibration(std::string ref_frame)
    :m_enableprint(false),
      m_printfile("estTraj.csv")
    {
        // ROS node generation
        ros::NodeHandle nh;
        
        _world = "world";
        // _detect_cam_frame = ref_frame;
        _detect_cam_frame = "camera_link";
        
        // Initialize camera tf 
        _camTf.header.frame_id = _world;
        // _camTf.child_frame_id = "camera_link";
        _camTf.child_frame_id = "estimated_link";

        setupTagList();
        setupTagTFMap();

        nh.param("/calibration/enableprint",m_enableprint,m_enableprint);
        nh.param("/calibration/printfile",m_printfile,m_printfile);

        if (remove(m_printfile.c_str()) != 0)
		    ROS_INFO("File deletion failed");
	    else
		    ROS_INFO("File deleted successfully");

    };


    void setupTagList()
    {
        _tagList.push_back("tag_0");
        _tagList.push_back("tag_1");
        _tagList.push_back("tag_2");
        _tagList.push_back("tag_3");
        _tagList.push_back("tag_4");
        
    }
    
    void setupTagTFMap()
    {
        Eigen::Affine3d _tw0, _t01, _t02, _t03, _t04;
        _tw0.setIdentity();
        _t01.setIdentity();
        _t02.setIdentity();
        _t03.setIdentity();
        _t04.setIdentity();

        // _t01.translation() = Eigen::Vector3d(0, 0.1, 0);
        // _t02.translation() = Eigen::Vector3d(0.09, 0.05, 0);
        // _t03.translation() = Eigen::Vector3d(0.195, 0, 0);
        // _t04.translation() = Eigen::Vector3d(0.195, 0.1, 0);

        // _tw0.translation() = Eigen::Vector3d(0.025, -0.32, 0);
        // _t01.translation() = Eigen::Vector3d(0.05, 0.1, 0);
        // _t02.translation() = Eigen::Vector3d(0.10, 0.0, 0);
        // _t03.translation() = Eigen::Vector3d(0.15, 0.1, 0);
        // _t04.translation() = Eigen::Vector3d(0.20, 0.0, 0);

        _tw0.translation() = Eigen::Vector3d(0.025, -0.32, 0);
        _t01.translation() = Eigen::Vector3d(0.075, 0.068, 0);
        _t02.translation() = Eigen::Vector3d(0.1250, -0.032, 0);
        _t03.translation() = Eigen::Vector3d(0.175, 0.068, 0);
        _t04.translation() = Eigen::Vector3d(0.2250, -0.032, 0);

        Eigen::Matrix3d m;
        m << 0, 1, 0, 
            -1, 0, 0,
            0, 0, 1;
        _tw0.linear() = m;
        _t01.linear() = m;
        _t02.linear() = m;
        _t03.linear() = m;
        _t04.linear() = m;

        ROS_INFO_STREAM(_tw0.matrix());
        _transforMap["tag_0"] = _tw0;
        _transforMap["tag_1"] = _t01;
        _transforMap["tag_2"] = _t02;
        _transforMap["tag_3"] = _t03;
        _transforMap["tag_4"] = _t04;

        

    }

    void eyehand(tf2_ros::Buffer& _tf_buffer)
    {

        geometry_msgs::TransformStamped transformStamped;

        // int idx = 0;
        int idx = 4;
        // ROS_INFO_STREAM(idx);
        // for (int i = 0; i < _tagList.size(); i++)
        for (int i = 4; i >=0 ; --i)  
        {
            try
            {
                
                // TF from tag to camera_link
                transformStamped = _tf_buffer.lookupTransform(_tagList[i], _detect_cam_frame, ros::Time(0), ros::Duration(0.1));
                idx = i;
                // ROS_INFO_STREAM(i);
                             
                break;
            }
            catch(tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }
        }
        // ROS_INFO_STREAM("index:" + std::to_string(idx));
        
        _prevTFTime = transformStamped.header.stamp;
        _prevTFt = transformStamped.transform.translation;
        geometry_msgs::Transform camT;
        camT = transformStamped.transform;
        
        Eigen::Affine3d cam_tag, world_cam;
        tf::transformMsgToEigen(camT, cam_tag);
        ROS_INFO_STREAM(_prevTFt);

        Eigen::Affine3d tw0;
        auto it = _transforMap.find("tag_0");
        tw0 = it->second;

        if(idx>0)
        {
            Eigen::Affine3d t0_ti;
            auto its = _transforMap.find(_tagList[idx]);
            t0_ti = its->second;
            world_cam = t0_ti * cam_tag;
            // world_cam = tw0 * t0_ti * cam_tag;
        }
        else
        {
            world_cam = tw0 * cam_tag;
            // world_cam = cam_tag;
        }
        // ROS_INFO_STREAM(idx);
        // ROS_INFO_STREAM(world_cam.matrix());
        publishCam(world_cam);

    };

    void outputFile(geometry_msgs::Vector3 ref_V)
    {
        std::ofstream out(m_printfile, std::ios_base::app);
        std::string line = std::to_string(ros::Time::now().toSec()) + ","+ 
            std::to_string(ref_V.x)+","+
            std::to_string(ref_V.y)+","+
            std::to_string(ref_V.z) + " \n";
        out<< line;
        out.close();
    }

    void publishCam(Eigen::Affine3d& matrix)
    {
        geometry_msgs::Transform camTrans;
        tf::transformEigenToMsg(matrix, camTrans);

        if(m_enableprint)
        {
            outputFile(camTrans.translation);
        }

        _camTf.header.stamp = ros::Time::now();
        _camTf.transform = camTrans;
        _br.sendTransform(_camTf);
    };


private:

    std::string _world, _detect_cam_frame;
    std::vector<std::string> _tagList;

    // TF broadcaster
    tf2_ros::TransformBroadcaster _br;
    geometry_msgs::TransformStamped _camTf;
    geometry_msgs::Vector3 _prevTFt;
    ros::Time _prevTFTime;
    std_msgs::Header _prevH;
    
    std::map<std::string, Eigen::Affine3d> _transforMap;

    std::string m_printfile;
    bool m_enableprint;


};

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "calibration");

    Calibration calibration(argv[1]);

    ros::Rate r(30);


    tf2_ros::Buffer tfbuffer; 
    tf2_ros::TransformListener tfListener(tfbuffer);


    while (ros::ok())
    {
        calibration.eyehand(tfbuffer);

        ros::spinOnce ();
        r.sleep();
    }

    return (0);
}

