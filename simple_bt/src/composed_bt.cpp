#include "behaviortree_cpp/action_node.h"
#include "/home/terabotics/behaviour_ws/src/simple_bt/include/crossdoor_nodes_.h"

inline void SleepMs(int ms){
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

BT::NodeStatus CrossDoor_::isDoorClosed(){
    SleepMs(200);
    if (!_door_close)
    {
        std::cout << "Door is open" << std::endl;
        // return BT::NodeStatus::FAILURE;
    }
    else

    {
        std::cout << "Door is closed" << std::endl;
        // return BT::NodeStatus::SUCCESS;
    }
    return !_door_close ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CrossDoor_::passThroughDoor(){
    SleepMs(200);
    std::cout << "Yeah!!!!!!! I'm passing through the door" << std::endl;
    return _door_open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus CrossDoor_::openDoor(){
    SleepMs(500);
    if (_door_locked)
    {
        std::cout << "OpenDoor Fallback: Door is locked" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    else
    {
    _door_open = true;
    std::cout << "OpenDoor Fallback: Door is open" << std::endl;
    return BT::NodeStatus::SUCCESS;
    }

}

BT::NodeStatus CrossDoor_::pickLock(){
    SleepMs(500);
    std::cout << "PickLock Fallback: Picking it up!" << std::endl;
    if (_pick_attempts++ >2)
    {
        _door_locked = false;
        _door_open = true;
        std::cout <<"PickLock Fallback: Yes I did it!" << std::endl;
    }   
    else
    {
        std::cout << _pick_attempts << "---- " <<"PickLock Fallback: I didn't pick it up!" << std::endl;
    }
    return _door_open ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    
}

BT::NodeStatus CrossDoor_::smashDoor(){
    SleepMs(500);
    _door_locked = false;
    _door_open = true;
    std::cout << "SmashDoor Fallback: Sorry had to Smash it" << std::endl;
    // smash always works
    return BT::NodeStatus::SUCCESS;
}
static const char* xml_text = R"(
    <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <Sequence>
                <Fallback>
                    <Inverter>
                        <IsDoorClosed/>
                    </Inverter>
                    <SubTree ID="DoorClosed"/>
                </Fallback>
                <PassThroughDoor/>
            </Sequence>
        </BehaviorTree>
        <BehaviorTree ID="DoorClosed">
            <Fallback>
                <OpenDoor/>
                <RetryUntilSuccessful num_attempts="5">
                    <PickLock/>
                </RetryUntilSuccessful>
                <SmashDoor/>
            </Fallback>
        </BehaviorTree>
    </root>
)";

int main(){
    BT::BehaviorTreeFactory factory;
    CrossDoor_ cross_door;
    cross_door.registerNodes(factory);
    factory.registerBehaviorTreeFromText(xml_text);
    auto tree = factory.createTree("MainTree");

    // std::string xml_models = BT::writeTreeNodesModelXML(factory);

    BT::printTreeRecursively(tree.rootNode());
    // std::cout << "--- ticking\n";
    // auto status = tree.tickOnce();
    // std::cout << "--- status: " << toStr(status) << "\n\n";
    tree.tickWhileRunning();
    return 0;  

}