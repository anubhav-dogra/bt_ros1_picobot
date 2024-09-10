#ifndef CONDITIONSNODES_DM_H
#define CONDITIONSNODES_DM_H


#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "ros/ros.h"
#include "ros/master.h"
#include <std_srvs/SetBool.h>
#include <geometry_msgs/WrenchStamped.h>
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

class IfComputeWrenchEnabled : public BT::ConditionNode
{   
public:
    IfComputeWrenchEnabled(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config), nh_("~")
    {
        // Initialize ROS Subscriber, just to check if the node is live or not! 
        // compute_wrench is actually publishing and subscribing to the topic. 
        // sub_ = nh_.subscribe("/cartesian_wrench_tool", 1, &IfComputeWrenchEnabled::wrenchCallback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {   
        // // ros::spinOnce(); // update callback
        // if ((ros::Time::now() - last_time_valid).toSec() < 1.0)
        // {
        //     // wrench_enabled_ = true;
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else
        // {
        //     // wrench_enabled_ = false;
        //     return BT::NodeStatus::FAILURE;
        // }
        // // return wrench_enabled_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        // To check if rosnode is available for ROS NODe list. 
        bool is_node_available = isNodeAvailable();
        return is_node_available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    bool wrench_enabled_ = false;
    ros::Time last_time_valid;
    bool is_node_available = false;
    std::vector<std::string> nodes;
    
    bool isNodeAvailable()
    {
        ros::master::getNodes(nodes);
        for(const auto& name : nodes)
        {
            if (name == "/get_wrench_base_sim")
            {
                return true;
            }
        }
        return false;
    }

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {   
        
        last_time_valid = ros::Time::now();
        // if (!msg)
        // {
        //     ROS_INFO("wrench callback: NULL");
        //     wrench_enabled_ = false;
        // }
        // else
        // {
        //     wrench_enabled_ = true;
        //     ROS_INFO("wrench callback");
        // }

        
    }

};

class IfRobotMotionEnabled : public BT::ConditionNode   
{
public:
    IfRobotMotionEnabled(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config), nh_("~")
    {
        
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }   

    BT::NodeStatus tick() override
    {
        // To check if rosnode is available for ROS NODe list. 
        bool is_node_available = isNodeAvailable();
        return is_node_available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }   


private:
    ros::NodeHandle nh_;
    std::vector<std::string> nodes;
    bool is_node_available = false;
    bool isNodeAvailable()
    {
        ros::master::getNodes(nodes);
        for(const auto& name : nodes)
        {
            if (name == "/plan_send_cartesian_commands")
            {
                return true;
            }
        }
        return false;
    }
};

class IfDetectionEnabled : public BT::ConditionNode
{
public:
    IfDetectionEnabled(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config), nh_("~")
    {
        
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }   

    BT::NodeStatus tick() override
    {
        // To check if rosnode is available for ROS NODe list. 
        bool is_node_available = isNodeAvailable();
        return is_node_available ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    ros::NodeHandle nh_;
    std::vector<std::string> nodes;
    bool is_node_available = false;
    bool isNodeAvailable()
    {
        ros::master::getNodes(nodes);
        for(const auto& name : nodes)
        {
            if (name == "/green_marker_detection_with_cropbox")
            {
                return true;
            }
        }
        return false;
    }

};

class IsPoseValid : public BT::ConditionNode
{
public:
    IsPoseValid(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config), nh_("~")
    {

        sub_ = nh_.subscribe("/tf_array_out_dZ", 10, &IsPoseValid::poseCallback, this);
        
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        // To check if rosnode is available for ROS NODe list. 
        if (pose_buffer.size() < min_buffer_size)
        {
            return BT::NodeStatus::FAILURE; // Not enough data yet
        }

        if(is_pose_stable())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::deque<geometry_msgs::PoseStamped> pose_buffer;
    int max_buffer_size = 20;
    int min_buffer_size = 10;
    double position_threshold = 0.005;
    void poseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped point_now_dZ;
        point_now_dZ.header.frame_id = "world";
        point_now_dZ.header.stamp = ros::Time::now();
        point_now_dZ.pose = msg->poses[0];
        pose_buffer.emplace_back(point_now_dZ);
        // Keep buffer size in limit
        if (pose_buffer.size() > max_buffer_size)
        {
            pose_buffer.pop_front();
        }
    }

    bool is_pose_stable()
    {
        const geometry_msgs::Pose& last_pose = pose_buffer.back().pose;
        const geometry_msgs::Pose& first_pose = pose_buffer.front().pose;

        double postition_difference = sqrt(pow(last_pose.position.x - first_pose.position.x, 2) + 
                                            pow(last_pose.position.y - first_pose.position.y, 2) +
                                            pow(last_pose.position.z - first_pose.position.z, 2));

        if (postition_difference < position_threshold)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

class AtHomePose : public BT::ConditionNode
{
public:
    AtHomePose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::ConditionNode(name, config), nh_("~")
    {
        // get current pose
        sub_ = nh_.subscribe("/tool_link_ee_pose", 10, &AtHomePose::CurrentPoseCallback, this);
        
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }   

    BT::NodeStatus tick() override
    {
        // To check if rosnode is available for ROS NODe list. 
        if(athomepose)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
       
    }

private:    
    ros::NodeHandle nh_;   
    ros::Subscriber sub_; 
    bool athomepose = false;
    void CurrentPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& msg)
    {

        double postition_difference = sqrt(pow(msg->transform.translation.x - (-0.62), 2) + 
                                            pow(msg->transform.translation.y - 0.0, 2) +
                                            pow(msg->transform.translation.z - 0.25, 2));

        if (postition_difference < 0.001)
        {
            athomepose = true;
        }
        else
        {
            athomepose = false;
        }
    }

};

#endif