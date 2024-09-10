BehaviorTree.CPP compiled using catkin build


# for compiling BT once.
cd BehaviorTree.cpp
mkdir build && cd build
cmake ..
sudo make install


# For compiling custom packages/files
cd directory
mkdir build && cd build 
cmake ..
make
./file_name # to execute


# ros based compiling now supported. 
cd bt_ws
# build simple_bt and autobot.
catkin build 