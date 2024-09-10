import rospy
from geometry_msgs.msg import PoseStamped
import os

# current_file_path= os.path.dirname(os.path.abspath(__file__))
# directory_path = os.path.dirname(current_file_path)
# src_path = os.path.dirname(directory_path)
src_path = "/home/terabotics/stuff_ws/src"
TIMEOUT = {'default': 60,
           'max': 1860}

# ROS Package and Launch Files
PACKAGE_NAME = 'tera_iiwa_ros'
LAUNCH_FILES = {
    'robot_motion': [src_path+'/tera_iiwa_ros/launch/init_robot_motion.launch'],
    'detection': [src_path+'/tera_iiwa_ros/launch/one_in_all.launch'],
    'wrench': [src_path+'/tera_iiwa_ros/launch/get_wrench_sim.launch'],
    'force_controller': [src_path+'/tera_iiwa_ros/launch/force_controller.launch'],
    'send_pose': [src_path+'/tera_iiwa_ros/launch/send_target_pose.launch'],
    'record_thz' : [src_path+'/tera_iiwa_ros/launch/terasmart_launch.launch' , 'file:=test', 'time:=60'],
    'record_bag': [src_path+'/tera_iiwa_ros/launch/record_bag.launch', 'file:=test'],
    'read_bag': [src_path+'/tera_iiwa_ros/launch/read_bag.launch', 'file:=test'],
}

# Home Pose
def get_home_pose():
    home_pose = PoseStamped()
    home_pose.header.frame_id = "world"
    home_pose.header.stamp = rospy.Time.now()
    home_pose.pose.position.x = -0.55
    home_pose.pose.position.y = 0
    home_pose.pose.position.z = 0.25
    home_pose.pose.orientation.x = 0.7
    home_pose.pose.orientation.y = 0.7
    home_pose.pose.orientation.z = 0
    home_pose.pose.orientation.w = 0
    return home_pose

# ROS Topics
TOPICS = {
    'robot_run': '/cartesian_trajectory_generator/new_goal',
    'force_topic': '/cartesian_wrench_tool',
    'current_pose': '/tool_link_ee_pose'
}


