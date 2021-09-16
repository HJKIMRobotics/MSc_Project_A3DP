#! /usr/bin/env python
import rospy
import rospkg
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import csv
import os
import numpy as np

# Spawn the cube model in gazebo
def spawn_model(path, pose, num):
    spawner = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    
    try:
        with open(path+ '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read()
    except IOError as err:
        rospy.logerr("Cannot find or open model, check model name and that model exists, I/O error message:  %s"%(err))
    except UnboundLocalError as error:
        rospy.logdebug("Cannot find package, check package name and that package exists, error message:  %s"%( error))
    

    item_name = "cube_{0}".format(num)

    res = spawner(item_name, model_xml, '',pose, 'world')
    if res.success == True:
        # SpawnModel: Successfully spawned entity 'model_name'
        rospy.loginfo(res.status_message + " " + item_name )
    else:
        rospy.logerr("Error: model %s not spawn, error message = "% item_name + res.status_message)

# Spawn the tag-background template in gazebo  
def spawn_tag(package):
    spawner = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    
    filepath = rospy.get_param('~plane_path', '/models/pattern/tagbackground')
    path = package + filepath

    try:
        with open(path+ '/model.sdf', 'r') as xml_file:
            model_xml = xml_file.read()
    except IOError as err:
        rospy.logerr("Cannot find or open model, check model name and that model exists, I/O error message:  %s"%(err))
    except UnboundLocalError as error:
        rospy.logdebug("Cannot find package, check package name and that package exists, error message:  %s"%( error))
    

    item_name = "tags_backgound"
    pose = Pose()
    pose.position.x = rospy.get_param('~plane_x', 0.0)
    pose.position.y = rospy.get_param('~plane_y', 0.0)
    # pose.position.z = 0.00
    ori = quaternion_from_euler(0, 0, 0)
    pose.orientation.x = ori[0]
    pose.orientation.y = ori[1]
    pose.orientation.z = ori[2]
    pose.orientation.w = ori[3]

    res = spawner(item_name, model_xml, '',pose, 'world')

def pose_publisher():
    
    # The way the camera moves
    pos_x = rospy.get_param('~init_x', 0.0)
    pos_y = rospy.get_param('~init_y', 0.0)
    hight = rospy.get_param('~init_z', 0.3)

    pattern = rospy.get_param('~traj_pattern', True)
    trav_x = rospy.get_param('~trav_x', 0.05)
    trav_y = rospy.get_param('~trav_y', 0.05)
    speed = rospy.get_param('~cam_speed', 0.0005)

    enable_noise = rospy.get_param('~enable_noise', False)
    std_x = rospy.get_param('~std_x', 0.001)
    std_y = rospy.get_param('~std_y', 0.001)
    std_z = rospy.get_param('~std_z', 0.001)

    max_iter = rospy.get_param('~max_iter', 1000)


    # Object spawner
    rospy.wait_for_service('gazebo/spawn_sdf_model',5.0)
    
    package_name = rospy.get_param('~package_name', 'msca3dp')
    relative_path = rospy.get_param('~relative_path', '/models/cube')
    pkgpath = rospkg.RosPack().get_path(package_name)
    complete_path = pkgpath + relative_path

    modelsize = rospy.get_param('~modelsize', 0.02)
    spawnObj = rospy.get_param('~spawnObj', False)

    obj_pose = Pose()
    obj_pose.position.z = 0.01
    ori = quaternion_from_euler(0, 0, 0)
    obj_pose.orientation.x = ori[0]
    obj_pose.orientation.y = ori[1]
    obj_pose.orientation.z = ori[2]
    obj_pose.orientation.w = ori[3]

    obj_num = 0
    trav_dis = 0

    # Tag background spawner
    enable_tag = rospy.get_param('~spawnTag', False)
    if(enable_tag):
        spawn_tag(pkgpath)
        rospy.Rate(30).sleep()

    # Pose and TF
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    
    t.header.frame_id = "world"
    t.child_frame_id = "base_link"
    q = quaternion_from_euler(0, 1.5707, 0)

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    t.transform.translation.z = hight

    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)
    pose_msg = ModelState()
    pose_msg.model_name = 'robot'

    pose_msg.pose.orientation.x = q[0]
    pose_msg.pose.orientation.y = q[1]
    pose_msg.pose.orientation.z = q[2]
    pose_msg.pose.orientation.w = q[3]

    pose_msg.pose.position.z = hight

    # Save tracj
    
    if os.path.exists("tracj.csv"):
        os.remove("tracj.csv")
    f = open("tracj.csv", mode="w")
    writer = csv.writer(f,delimiter=',',quotechar='|', quoting=csv.QUOTE_NONE)

    rate = rospy.Rate(30)

    if pattern:
        
        horizon = True
        up = True

        while not rospy.is_shutdown() :

            if horizon:
                pos_x += speed

            # python math modulo remainder calculation
            # sometimes can't get 0.0
            if pos_x % trav_x < speed:
                horizon = False
                if up:
                    pos_y += speed
                    er = round(pos_y,5) % trav_y
                    if er < speed:
                        up = False
                        horizon = True
                else:
                    pos_y -= speed
                    er = round(pos_y,5) % trav_y
                    if er < speed:
                        up = True
                        horizon = True

            if(enable_noise):
                # x_noise =  np.random.normal(0,std_x)
                # y_noise = np.random.normal(0,std_y)
                z_noise = hight + np.random.normal(0,std_z)
                
                # pos_x += x_noise
                # pos_y += y_noise
                pose_msg.pose.position.z = z_noise
                t.transform.translation.z = z_noise
            
            pose_msg.pose.position.x = pos_x
            pose_msg.pose.position.y = pos_y
            pub.publish(pose_msg)

            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = pos_x
            t.transform.translation.y = pos_y
            br.sendTransform(t)

            if spawnObj:
                if trav_dis % modelsize < speed:
                    obj_pose.position.x = pos_x
                    obj_pose.position.y = pos_y
                    spawn_model(complete_path,obj_pose,obj_num)
                    obj_num += 1
                trav_dis += speed
            
            
            # write position to file
            writer.writerow([str(pos_x), str(pos_y), str(hight)])
            rate.sleep()
    else:
        while not rospy.is_shutdown():

            if(enable_noise):
                x_noise =  np.random.normal(0,std_x)
                y_noise = np.random.normal(0,std_y)
                z_noise = hight + np.random.normal(0,std_z)
                
                pos_x += speed + x_noise
                pos_y = y_noise

                pose_msg.pose.position.y = pos_y
                pose_msg.pose.position.z = z_noise

                t.transform.translation.y = pos_y
                t.transform.translation.z = z_noise
            else:
                pos_x += speed

            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = pos_x

            pose_msg.pose.position.x = pos_x
            pub.publish(pose_msg)

            br.sendTransform(t)
            rate.sleep()
    

 
if __name__ == '__main__':
    rospy.init_node('pose_publisher')

    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass