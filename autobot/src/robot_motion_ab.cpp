#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/SetBool.h"


class RobotMotionNode {
public:
    RobotMotionNode(){
        sub_ = nh_.subscribe("/cartesian_trajectory_generator/ref_pose", 10, &RobotMotionNode::poseCallback, this);
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose", 10);
        srv_ = nh_.advertiseService("/enable_robot_motion", &RobotMotionNode::enableMotionCallback, this);
        enabled_ = false; //start disabled
    }

    bool enableMotionCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
        
        enabled_ = req.data;
        res.success = true;
        res.message = enabled_ ? "Motion enabled" : "Motion disabled";

        return true;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (enabled_){
            pose_ = *msg;
            pub_.publish(pose_);
        }
        else {
            ROS_INFO("Robot Motion is disabled. Skipping pose update.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::ServiceServer srv_;
    geometry_msgs::PoseStamped pose_;
    bool enabled_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_motion_service");
    RobotMotionNode robot_motion_node;
    ros::spin();
    return 0;
}

