#ifndef  ROBOT_MOTION_H
#define  ROBOT_MOTION_H

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>


class EnableRobotMotion : public BT::SyncActionNode{
public:
    EnableRobotMotion(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
        // Initialize ROS Client
        nh_ = ros::NodeHandle();
        motion_service_client_ = nh_.serviceClient<std_srvs::SetBool>("/enable_robot_motion");
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<bool>("enable")};
    }

    BT::NodeStatus tick() override
    {
        bool enable;
        if (!getInput<bool>("enable", enable))
        {
            throw BT::RuntimeError("missing required input [enable]: " + std::to_string(enable));
        }
        std_srvs::SetBool srv;
        srv.request.data = enable;

        if (motion_service_client_.call(srv))
        {
            ROS_INFO("%s", srv.response.message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_ERROR("Failed to start robot motion node.");
            return BT::NodeStatus::FAILURE;
        }
    }

private: 
    ros::NodeHandle nh_;
    ros::ServiceClient motion_service_client_;
};

class ValidateTargetPose : public BT::ConditionNode{

    public:
        ValidateTargetPose(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::ConditionNode(name, config), nh_vp("~")
        {
            pose_sub = nh_vp.subscribe("/target_pose", 1, &ValidateTargetPose::targetPoseCallback, this);
        }

        BT::NodeStatus tick() override
        {
            ros::spinOnce(); // update callback
            if (!target_pose_is_valid)  
            {
                return BT::NodeStatus::FAILURE;
            }
            else
            {
                target_pose_is_valid = false; // Reset the flag after checking
                return BT::NodeStatus::SUCCESS;
            }
            // return target_pose_is_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }
        
        static BT::PortsList providedPorts()
        {
            return BT::PortsList();
        }

    private:
        ros::NodeHandle nh_vp;
        ros::Subscriber pose_sub;
        bool target_pose_is_valid = false;

        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            // ROS_INFO_STREAM("Target pose received: " << msg->pose.position.x << " " << msg->pose.position.y << " " << msg->pose.position.z);
            
            target_pose_is_valid = msg->pose.position.z != 0;
        }
};

class SendTargetPose : public BT::SyncActionNode
{

    public:
        SendTargetPose(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::SyncActionNode(name, config), nh_sp("~")
        {
            pose_pub = nh_sp.advertise<geometry_msgs::PoseStamped>("/cartesian_trajectory_generator/new_goal", 1);
            pose_sub = nh_sp.subscribe("/target_pose", 1, &SendTargetPose::targetPoseCallback, this);
        }

        BT::NodeStatus tick() override  
        {
            if (!msg_came)
            {
                return BT::NodeStatus::FAILURE;
            }

            pose_pub.publish(pose);
            return BT::NodeStatus::SUCCESS;
        }

        static BT::PortsList providedPorts()
        {
            return BT::PortsList();
        }

    private:
        ros::NodeHandle nh_sp;
        ros::Publisher pose_pub;
        ros::Subscriber pose_sub;
        geometry_msgs::PoseStamped pose;
        bool msg_came = false;
        void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            // check if msg is null
            if (!msg)   
            {
                msg_came = false;
                ROS_INFO("Target pose received: NULL");
                return;
            }
            else
            {
                msg_came = true;
            }
            pose.header = msg->header;
            pose.pose = msg->pose;
        }
};
#endif