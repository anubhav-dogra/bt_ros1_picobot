#include <iostream>
#include <chrono>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "ros/ros.h"

using namespace std::chrono_literals;

//Node class
class ApproachObject : public BT::SyncActionNode
{
public:
    ApproachObject(const std::string& name)
      : BT::SyncActionNode(name, {})
    {
    }

    BT::NodeStatus tick() override
    {
        std::cout << "Approach the object" << this->name() << std::endl;
        std::this_thread::sleep_for(3s);
        return BT::NodeStatus::SUCCESS;
    }
};

// Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
    std::cout << "Checking battery" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// We want to wrap into an ActionNode the methods open() and close()
class GripperInterface
{
public:
  GripperInterface(): _open(true) {}
    
  BT::NodeStatus open() 
  {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close() 
  {
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool _open; // shared information
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_action_node_cpp");
    ros::NodeHandle nh;
    // We use the BehaviorTreeFactory to register our custom nodes
    BT::BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    factory.registerNodeType<ApproachObject>("ApproachObject");

    // Registering a SimpleActionNode using a function pointer.
    // You can use C++11 lambdas or std::bind
    factory.registerSimpleCondition("CheckBattery", [&](BT::TreeNode&) { return CheckBattery(); });
    // factory.registerSimpleCondition("CheckBattery",std::bind(CheckBattery));

    //You can also create SimpleActionNodes using methods of a class
    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", [&](BT::TreeNode&){ return gripper.open(); } ); // VSCODE SHOWS ERROR: BUT THE CODE IS SILL RUNNING. !!!!!!!!!!!
    factory.registerSimpleAction("CloseGripper", [&](BT::TreeNode&){ return gripper.close(); } );
    




    // Trees are created at deployment-time (i.e. at run-time, but only 
    // once at the beginning). 
    
    // IMPORTANT: when the object "tree" goes out of scope, all the 
    // TreeNodes are destroyed
    static const char* xml_text = R"(
                                        <root BTCPP_format="4" >
                                            <BehaviorTree ID = "MainTree">
                                                <Sequence name = "root_sequence">
                                                    <CheckBattery name = "check_battery"/>
                                                    <OpenGripper name = "open_gripper"/>
                                                    <ApproachObject name = "approach_object"/>
                                                    <CloseGripper name = "close_gripper"/>
                                                </Sequence>                                            
                                            </BehaviorTree>
                                        </root>
                                        )";


    auto tree = factory.createTreeFromText(xml_text);
    tree.tickWhileRunning();

    return 0;
 
}