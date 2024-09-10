#### Install the required dependencies:
Open a terminal and run the following commands to install the necessary 
dependencies:
```
sudo apt-get update
sudo apt-get install -y libzmq3-dev libboost-dev libboost-system-dev libboost-filesystem-dev libboost-thread-dev libprotobuf-dev protobuf-compiler libmsgsl-dev libgtest-dev cmake
```
#### Build and install BehaviorTree.CPP:
```
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install
```
Now build autobot and simple_bt libraries
```
cd bt_ws/src
catkin_build
```

#### To Run
- `cd bt_ws`
- First run the robot, trajectory planner, camera etc. (may be from seperate terminals)
- Then run server_node using `python3 autobot/scripts/server_node.py`
- Then run the force action server `rosrun autobot force_controller_action`
- Then run the BT using `rosrun simple_bt decision_maker`


#### Changes
if neccessary, make changes to the files. 
Such as:
- change the launch file name in the config.py file in the autobot folder (get_wrench_sim to get_wrench)
- 
TADAA
