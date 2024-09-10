cmake_minimum_required(VERSION 3.5)

project(simple_bt)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(behaviortree_cpp)
find_package(roscpp REQUIRED) 

include_directories(${roscpp_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_simple action_node.cpp)  
add_executable(${PROJECT_NAME}_port port_example.cpp) 
add_executable(${PROJECT_NAME}_port_generic port_generic.cpp)    
add_executable(${PROJECT_NAME}_rb reactive_behavior.cpp)
add_executable(composed_bt composed_bt.cpp)

target_link_libraries(${PROJECT_NAME}_simple ${roscpp_LIBRARIES} BT::behaviortree_cpp)
target_link_libraries(${PROJECT_NAME}_port BT::behaviortree_cpp)
target_link_libraries(${PROJECT_NAME}_port_generic BT::behaviortree_cpp)
target_link_libraries(${PROJECT_NAME}_rb BT::behaviortree_cpp)
target_link_libraries(composed_bt BT::behaviortree_cpp)
