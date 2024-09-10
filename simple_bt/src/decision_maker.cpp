#include <ros/ros.h>
// #include "/home/terabotics/behaviour_ws/src/simple_bt/include/robot_motion.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "../include/conditionsnodes_dm.h"
#include "../include/actionnodes_dm.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"
#include "fstream"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle nh;
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<IfRobotMotionEnabled>("IfRobotMotionEnabled");
    factory.registerNodeType<EnableRobotMotion>("EnableRobotMotion");
    factory.registerNodeType<AtHomePose>("AtHomePose");
    factory.registerNodeType<SendHomePose>("SendHomePose");
    factory.registerNodeType<IfComputeWrenchEnabled>("IfComputeWrenchEnabled");
    factory.registerNodeType<EnableComputeWrench>("EnableComputeWrench");
    factory.registerNodeType<IfDetectionEnabled>("IfDetectionEnabled");
    factory.registerNodeType<EnableDetection>("EnableDetection");
    factory.registerNodeType<WaitPeriod>("WaitPeriod");
    factory.registerNodeType<IsPoseValid>("IsPoseValid");
    factory.registerNodeType<SendTargetPose>("SendTargetPose");
    factory.registerNodeType<ActivateForceController>("ActivateForceController");

    // factory.registerNodeType<SendTargetPose>("SendTargetPose");
    // std::string xml_models = BT::writeTreeNodesModelXML(factory);
    // // save to xml_models to a file
    // std::ofstream ofs("/home/terabotics/behaviour_ws/src/simple_bt/behavior_trees/decision_maker_groot.xml");
    // ofs << xml_models;
    // ofs.close();
     
    // std::string xml_text = R"(
    // <root BTCPP_format="4">
    //     <BehaviorTree>
    //         <Sequence name ="root">
    //             <EnableRobotMotion name ="EnableRobotMotion" enable="true"/>
    //             <ValidateTargetPose name ="ValidateTargetPose"/>
    //             <SendTargetPose name ="SendTargetPose"/>
    //         </Sequence>
    //     </BehaviorTree>
    // </root>
    // )";

    // auto tree = factory.createTreeFromText(xml_text);
    //  get current directory
    // std::cout << "Current working directory: " << std::filesystem::current_path() << std::endl;
    auto tree = factory.createTreeFromFile("/home/terabotics/behaviour_ws/src/simple_bt/behavior_trees/decision_maker.xml");
    // const unsigned port = 5555;
    // BT::Groot2Publisher publisher(tree,port);
    // BT::FileLogger2 logger2(tree, "t12_logger2.btlog");
    // BT::MinitraceLogger minilog(tree, "minitrace.json");

    BT::StdCoutLogger logger_cout(tree);
    BT::printTreeRecursively(tree.rootNode());
    // BT::NodeStatus status = tree.tickOnce();
    ros::Rate rate(10);
    while (ros::ok())
        {
            BT::NodeStatus status = tree.tickWhileRunning();
            if (status == BT::NodeStatus::SUCCESS)
            {
                break;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();  // Small delay to avoid busy-waiting
        }
    // while (ros::ok())
    // {
    //     BT::NodeStatus status = tree.tickOnce();
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    ros::spin();
   
    return 0;
}