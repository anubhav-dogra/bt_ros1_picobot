#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"


// INPUT PORTS
class SaySomething : public BT::SyncActionNode
{
public:
    // If your Node has ports, you must use this constructor signature 
    SaySomething(const std::string& name, const BT::NodeConfig& config) 
    : BT::SyncActionNode(name, config)
    {}

    // It is mandatory to define this STATIC method.
    static BT::PortsList providedPorts()
    {
        // This action has a single input port called "message"
        return {BT::InputPort<std::string>("message")};
    }
    // Override the virtual function tick()
    BT::NodeStatus tick() override
    {
        BT::Expected<std::string> msg = getInput<std::string>("message");
        if(!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }
        // use the method value() to extract the valid message.
        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// OUTPUT PORTS
class ThinkWhatToSay : public BT::SyncActionNode
{
    public:
        ThinkWhatToSay(const std::string& name, const BT::NodeConfig& config)
          : BT::SyncActionNode(name, config)
        {}
        static BT::PortsList providedPorts()
        {
          return { BT::OutputPort<std::string>("text") };
        }
        // This Action simply write a value in the port "text"
        BT::NodeStatus tick() override
        {
          setOutput("text", "Here is my answer");
          return BT::NodeStatus::SUCCESS;
        }
};

int  main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
    auto tree = factory.createTreeFromText(R"(
      <root BTCPP_format="4" >
        <BehaviorTree ID="MainTree">
            <Sequence name="root_sequence">
                <SaySomething     message="hello" />
                <ThinkWhatToSay   text="{the_answer}"/>
                <SaySomething     message="{the_answer}" />
            </Sequence>
        </BehaviorTree>
</root>
    )");
    tree.tickWhileRunning();
    return 0;
}