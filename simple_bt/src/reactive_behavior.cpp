#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include <chrono>


struct Pose2D
{
    double x, y, theta;
};
template <> inline Pose2D BT::convertFromString(StringView str)
{
    auto parts = BT::splitString(str, ';');
    if(parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}
class MoveBaseAction : public BT::StatefulActionNode
{
public: 
    MoveBaseAction(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {
    }

    // this function is invoked once at the beginning.
    static BT::PortsList providedPorts(){
        return {
            BT::InputPort<Pose2D>("goal")
        };
    }

    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:
    Pose2D _goal;
    std::chrono::system_clock::time_point _completion_time;
};

//-------------------------------------------------------------------------------

BT::NodeStatus MoveBaseAction::onStart()
{
    if (!getInput<Pose2D>("goal",_goal))
    {
        throw BT::RuntimeError("missing required input [goal]: ");
    }
    printf("MoveBase: Send Request]. goal: x=%f y=%f theta=%f\n", _goal.x, _goal.y, _goal.theta);
    
    // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)
    _completion_time = std::chrono::system_clock::now() + std::chrono::milliseconds(220);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (std::chrono::system_clock::now() >= _completion_time)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
    printf("[ MoveBase: ABORTED ]");
}

BT::NodeStatus BatteryOK()
{
    std::cout << "Checking battery" << std::endl;
    return BT::NodeStatus::SUCCESS;
}


class SaySomething : public BT::SyncActionNode
{
public:
    SaySomething(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message")};
    }   

    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> msg = getInput<std::string>("message");
        if(!msg)    
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        std::cout << "Robot says: " <<  msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};  

static const char* xml_text = R"(
    <root BTCPP_format="4">
     <BehaviorTree>
        <ReactiveSequence>
            <BatteryOK/>
            <SaySomething   message="mission started..." />
            <MoveBase           goal="1;2;3"/>
            <SaySomething   message="mission completed!" />
        </ReactiveSequence>
     </BehaviorTree>
    </root>
    )";

int main()
{

    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", [&](BT::TreeNode&) { return BatteryOK(); });
    factory.registerNodeType<MoveBaseAction>("MoveBase"); // register the ActionNode>
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromText(xml_text);

        // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking\n";
    auto status = tree.tickOnce();
    std::cout << "--- status: " << toStr(status) << "\n\n";

    while(status == BT::NodeStatus::RUNNING) 
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }

  return 0;
}