#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

struct Position2D
{
    double x;
    double y;
};
template <> inline Position2D BT::convertFromString(StringView str)
{
    auto parts = BT::splitString(str, ';');
    if(parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Position2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        return output;
    }
}
class CalculateGoal : public BT::SyncActionNode
{
public:
    CalculateGoal(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<Position2D>("goal")};
    }
    BT::NodeStatus tick() override
    {
        Position2D mygoal = { 1.1, 2.3 };
        setOutput<Position2D>("goal",  mygoal);
        return BT::NodeStatus::SUCCESS;
    }
};

class PrintTarget : public BT::SyncActionNode
{
public:
    PrintTarget(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
    {}
    static BT::PortsList providedPorts()
    {   
        const char* description = "Simply print the goal on console...";
        return {BT::InputPort<Position2D>("target", description)};
    }
    BT::NodeStatus tick() override
    {
        auto res = getInput<Position2D>("target");
        if (!res)
        {
            throw BT::RuntimeError("error reading port [target]: ", res.error());
        }
        Position2D target = res.value();
        printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y);
        return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <CalculateGoal  goal="{GoalPosition}"/>
                <PrintTarget    target="{GoalPosition}"/>
                <Script         code="OtherGoal='-1;3'"/>
                <PrintTarget    target="{OtherGoal}"/>
            </Sequence>
        </BehaviorTree>
    </root>
    )";

int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");
    
    auto tree = factory.createTreeFromText(xml_text);

    tree.tickWhileRunning();
    return 0;

}