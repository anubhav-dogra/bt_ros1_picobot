cmake_minimum_required(VERSION 3.5)
project(simple_bt)

set(CMAKE_CXX_STANDARD 17) # issue with C++14 while compiling. 
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(behaviortree_cpp REQUIRED)
# set(BTCPP_INCLUDE_DIR "/home/{username}/behaviour_ws/src/BehaviorTree.CPP/include")
# set(BTCPP_LIB "/home/{username}/behaviour_ws/src/BehaviorTree.CPP/build/libbehaviortree_cpp.so")


find_package(catkin REQUIRED COMPONENTS
 roscpp
 behaviortree_cpp 
 cartesian_trajectory_generator
 autobot) 

catkin_package( 
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp behaviortree_cpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(simple_ros_bt src/action_node.cpp)
add_executable(composed_bt src/composed_bt.cpp)
add_executable(decision_maker src/decision_maker.cpp)

target_link_libraries(simple_ros_bt ${catkin_LIBRARIES})
target_link_libraries(composed_bt ${catkin_LIBRARIES})
target_link_libraries(decision_maker ${catkin_LIBRARIES})

