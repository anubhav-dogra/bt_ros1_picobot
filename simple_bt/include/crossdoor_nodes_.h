#pragma once
#include <behaviortree_cpp/bt_factory.h>

class CrossDoor_
{
public:
    void registerNodes(BT::BehaviorTreeFactory& factory);

    void reset();

    BT::NodeStatus isDoorClosed();

    BT::NodeStatus passThroughDoor();

    BT::NodeStatus openDoor();

    BT::NodeStatus pickLock();

    BT::NodeStatus smashDoor();

private:
    bool _door_close = true;
    bool _door_open = false;
    bool _door_locked = true;
    int _pick_attempts = 0;
    
};
void CrossDoor_::registerNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerSimpleCondition(
      "IsDoorClosed", std::bind(&CrossDoor_::isDoorClosed, this));

  factory.registerSimpleAction(
      "PassThroughDoor", std::bind(&CrossDoor_::passThroughDoor, this));

  factory.registerSimpleAction(
      "OpenDoor", std::bind(&CrossDoor_::openDoor, this));

  factory.registerSimpleAction(
      "PickLock", std::bind(&CrossDoor_::pickLock, this));

  factory.registerSimpleCondition(
      "SmashDoor", std::bind(&CrossDoor_::smashDoor, this));
}
