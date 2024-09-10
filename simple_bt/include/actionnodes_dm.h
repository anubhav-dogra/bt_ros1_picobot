#ifndef ACTIONNODES_DM_H
#define ACTIONNODES_DM_H

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseStamped.h>
#include "actionlib/client/simple_action_client.h"
#include "cartesian_trajectory_generator/TrajectoryAction.h"
#include "autobot/ForceControlAction.h"

class EnableComputeWrench : public BT::SyncActionNode
{
public:
    EnableComputeWrench(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config), nh_("~")
    {
        // Initialize ROS Client
        wrench_service_client_ = nh_.serviceClient<std_srvs::Trigger>("/wrench_enabler");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }   

    BT::NodeStatus tick() override  
    {
        ros::spinOnce();
        
        if (!wrench_service_client_.exists())
        {
            ROS_ERROR("Service /start_launch is not available.");
            return BT::NodeStatus::FAILURE;
        }
        
        std_srvs::Trigger srv;

        if (wrench_service_client_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("%s", srv.response.message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
        
            else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
    }

private: 
    ros::NodeHandle nh_;
    ros::ServiceClient wrench_service_client_;
};

class EnableRobotMotion : public BT::SyncActionNode
{
public:
    EnableRobotMotion(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config), nh_("~")
    {
        // Initialize ROS Client
        robot_motion_service_client_ = nh_.serviceClient<std_srvs::Trigger>("/robot_motion_enabler");
    }   

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ros::spinOnce();
        
        if (!robot_motion_service_client_.exists())
        {
            ROS_ERROR("Service /robot_motion_enabler is not available.");
            return BT::NodeStatus::FAILURE;
        }
        
        std_srvs::Trigger srv;

        if (robot_motion_service_client_.call(srv))
        {
            if (srv.response.success)
            {
                ROS_INFO("%s", srv.response.message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
        
            else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
    }

private: 
    ros::NodeHandle nh_;
    ros::ServiceClient robot_motion_service_client_;
};

class EnableDetection : public BT::SyncActionNode
{
public:
    EnableDetection(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config), nh_("~")
    {
        // Initialize ROS Client
        detection_service_client_ = nh_.serviceClient<std_srvs::Trigger>("/detection_enabler");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        ros::spinOnce();
        
        if (!detection_service_client_.exists())
        {
            ROS_ERROR("Service /detection_enabler is not available.");
            return BT::NodeStatus::FAILURE;
        }
        
        std_srvs::Trigger srv;

        if (detection_service_client_.call(srv))    
        {
            if (srv.response.success)
            {
                ROS_INFO("%s", srv.response.message.c_str());
                return BT::NodeStatus::SUCCESS;
            }
        
            else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        else
            {
                ROS_ERROR("%s", srv.response.message.c_str());
                return BT::NodeStatus::FAILURE;
            }
    }   

private: 
    ros::NodeHandle nh_;
    ros::ServiceClient detection_service_client_;
};

class WaitPeriod : public BT::SyncActionNode
{
public:
    WaitPeriod(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config), nh_("~")
    {
        
    }   

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        _completion_time = std::chrono::system_clock::now() + std::chrono::seconds(5);

        while (std::chrono::system_clock::now() < _completion_time)
        {
            ros::spinOnce();
        }
        return BT::NodeStatus::SUCCESS;

    }     

private: 
    ros::NodeHandle nh_;
    std::chrono::system_clock::time_point _completion_time;
};

class SendHomePose : public BT::StatefulActionNode
{
public:
    
    SendHomePose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config), action_client_("cartesian_trajectory_generator/goal_action", true)
    {
        ROS_INFO("Waiting for action server to start sending home Pose.");
        action_client_.waitForServer();
        ROS_INFO("Action server started, sending goal for Home Pose.");
        
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private: 
    ros::NodeHandle nh_;
    using ActionClient = actionlib::SimpleActionClient<cartesian_trajectory_generator::TrajectoryAction>;
    ActionClient action_client_;
    std::chrono::system_clock::time_point _completion_time;
};

class SendTargetPose : public BT::StatefulActionNode
{   
public:
    SendTargetPose(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config), action_client_("cartesian_trajectory_generator/goal_action", true)
    {
        ROS_INFO("Waiting for Target pose action server to start.");
        action_client_.waitForServer();
        ROS_INFO("Action server started, sending target Pose goal Soon.");
        sub_ = nh_.subscribe("/tf_array_out_dZ", 10, &SendTargetPose::poseCallback, this);  
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private: 
    ros::NodeHandle nh_;
    using ActionClient = actionlib::SimpleActionClient<cartesian_trajectory_generator::TrajectoryAction>;
    ActionClient action_client_;
    std::chrono::system_clock::time_point _completion_time;
    ros::Subscriber sub_;
    geometry_msgs::PoseStamped target_pose_;
    void poseCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
        target_pose_.header.frame_id = "world";
        target_pose_.header.stamp = ros::Time::now();
        target_pose_.pose = msg->poses[0];
    }
};

class ActivateForceController : public BT::StatefulActionNode
{
public:
    ActivateForceController(const std::string& name, const BT::NodeConfiguration& config)
      : BT::StatefulActionNode(name, config), action_client_("/force_control", true)
    {
        // force_controller_client_ = nh_.serviceClient<std_srvs::Trigger>("/force_controller_enabler");
        ROS_INFO("Waiting for Force action server to start.");
        action_client_.waitForServer();
        ROS_INFO("Action server started, sending Force goal.");
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    // BT::NodeStatus tick() override  
    // {
    //     ros::spinOnce();
        
        // if (!force_controller_client_.exists())
        // {
        //     ROS_ERROR("Service /force_controller_enabler is not available.");
        //     return BT::NodeStatus::FAILURE;
        // }
        // std_srvs::Trigger srv;
        // if (force_controller_client_.call(srv))    
        // {
        //     if (srv.response.success)
        //     {
        //         ROS_INFO("%s", srv.response.message.c_str());
        //         return BT::NodeStatus::SUCCESS;
        //     }
        
        //     else
        //     {
        //         ROS_ERROR("%s", srv.response.message.c_str());
        //         return BT::NodeStatus::FAILURE;
        //     }
        // }
        // else
        //     {
        //         ROS_ERROR("%s", srv.response.message.c_str());
        //         return BT::NodeStatus::FAILURE;
        //     }
    // }

private: 
    ros::NodeHandle nh_;    
    using ActionClient = actionlib::SimpleActionClient<autobot::ForceControlAction>;
    ActionClient action_client_;
};

BT::NodeStatus SendHomePose::onStart(){
    cartesian_trajectory_generator::TrajectoryGoal goal;
    goal.goal.header.stamp = ros::Time::now();
    goal.goal.header.frame_id = "world";
    goal.goal.pose.position.x = - 0.62;
    goal.goal.pose.position.y = 0.0;
    goal.goal.pose.position.z = 0.25;
    goal.goal.pose.orientation.w = 0.0;
    goal.goal.pose.orientation.x = 0.7;
    goal.goal.pose.orientation.y = 0.7;
    goal.goal.pose.orientation.z = 0.0;


    action_client_.sendGoal(goal);

    // _completion_time = std::chrono::system_clock::now() + std::chrono::seconds(10);
    // goal.trajectory_type = cartesian_trajectory_generator::TrajectoryGoal::HOME;
    // action_cleint_.sendGoal(goal);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SendHomePose::onRunning(){

    // action_cleint_.getState();
    actionlib::SimpleClientGoalState state = action_client_.getState();

    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(30.0)); // Timeout after 30 seconds

    if (finished_before_timeout)
    {
        if(state.isDone())
        {
            auto result = action_client_.getResult();
            if(result->error_code == cartesian_trajectory_generator::TrajectoryResult::SUCCESSFUL){
                
                ROS_INFO("Action finished: %s", result->error_string.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Action failed: %s", result->error_string.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        
        else
        {   
            // cartesian_trajectory_generator::TrajectoryFeedback feedback;
            
            // ROS_INFO("Feedback:%f", feedback.time_percentage);

            return BT::NodeStatus::RUNNING;
        }

        
    }
    else
    {
        ROS_INFO("Action did not finish in time.");
        return BT::NodeStatus::FAILURE;
    }       
}

void SendHomePose::onHalted(){
    action_client_.cancelAllGoals();
    printf("[Home Position Halted]");
}

BT::NodeStatus SendTargetPose::onStart(){
    cartesian_trajectory_generator::TrajectoryGoal goal;
    if(!target_pose_.header.frame_id.empty()){
        goal.goal = target_pose_;
        action_client_.sendGoal(goal);
        return BT::NodeStatus::RUNNING;
    }
    else
    {
        ROS_INFO("Target pose not received");
        return BT::NodeStatus::FAILURE;
    }
    
}

BT::NodeStatus SendTargetPose::onRunning(){
    actionlib::SimpleClientGoalState state = action_client_.getState();

    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(30.0)); // Timeout after 30 seconds

    if (finished_before_timeout)
    {
        if(state.isDone())
        {
            auto result = action_client_.getResult();
            if(result->error_code == cartesian_trajectory_generator::TrajectoryResult::SUCCESSFUL){
                
                ROS_INFO("Action finished: %s", result->error_string.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                ROS_INFO("Action failed: %s", result->error_string.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
        
        else
        {   
            // cartesian_trajectory_generator::TrajectoryFeedback feedback;
            
            // ROS_INFO("Feedback:%f", feedback.time_percentage);

            return BT::NodeStatus::RUNNING;
        }

        
    }
    else
    {
        ROS_INFO("Action did not finish in time.");
        return BT::NodeStatus::FAILURE;
    }       
}

void SendTargetPose::onHalted(){
    action_client_.cancelAllGoals();
    printf("[Home Position Halted]");
}

BT::NodeStatus ActivateForceController::onStart(){
    autobot::ForceControlGoal goal;
    goal.desired_force = 6.0;
    goal.duration = 30.0;
    action_client_.sendGoal(goal);
    // check if action is alive    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ActivateForceController::onRunning(){
    actionlib::SimpleClientGoalState state = action_client_.getState();
    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(35.0)); // Timeout after 30 seconds
      
    // if (finished_before_timeout)
    // {
    if(state.isDone())
    {
        auto result = action_client_.getResult();
        if(result->success == true){
            
            ROS_INFO("Action finished");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            ROS_INFO("Action failed");
            return BT::NodeStatus::FAILURE;
        }
    }         
    // }
    // else
    // {
    //     ROS_INFO("Action did not finish in time.");
    //     return BT::NodeStatus::FAILURE;
    // }   

    autobot::ForceControlFeedback feedback;
    ROS_INFO("Feedback:%f", feedback.current_force);            
    // ROS_INFO("Feedback:%f", feedback.time_percentage);

    return BT::NodeStatus::RUNNING;    
}   

void ActivateForceController::onHalted(){
    action_client_.cancelAllGoals();
    printf("[Force Controller Halted]");
}

#endif 