import rospy
import roslaunch
from ros_interface import RosLaunchProcess, ROSInterface
from config import *
from std_srvs.srv import Trigger, TriggerResponse
from multiprocessing import Queue

class ServerLauncher():
    def __init__(self):

        rospy.init_node('server_node', anonymous=True)
        self.uuid = rospy.get_param("/run_id")
        self.processes = {}
        self.status_queue = Queue()
        self._server_wrench = rospy.Service('/wrench_enabler', Trigger, self.wrench_enabler_callback)
        self._server_motion = rospy.Service('/robot_motion_enabler', Trigger, self.robot_motion_enabler_callback)
        self._server_detection = rospy.Service('/detection_enabler', Trigger, self.detection_enabler_callback)
        # self._server_force = rospy.Service('/force_controller_enabler', Trigger, self.force_controller_enabler_callback)
        rospy.loginfo("Launch service node is ready to start launch files.")

    def wrench_enabler_callback(self, req):
        if 'wrench' in self.processes:
            rospy.logwarn("A wrench launch file is already running.")
            return TriggerResponse(
                success=False,
                message="Launch file is already running."
            )

        try:
            self.toggle_process(True, 'wrench', LAUNCH_FILES['wrench'])
            rospy.loginfo("Wrench Launch file started")
            return TriggerResponse(
                success=True,
                message="Launch file started successfully."
            )

        except Exception as e:
            rospy.logerr("Failed to start launch file: {}".format(e))
            return TriggerResponse(
                success=False,
                message="Failed to start launch file."
            )

    ######## Robot Motion Enabler ########################################
    def robot_motion_enabler_callback(self, req):
        if 'robot_motion' in self.processes:
            rospy.logwarn("A robot motion launch file is already running.")
            return TriggerResponse(
                success=False,
                message="Launch file is already running."
            )

        try:
            self.toggle_process(True, 'robot_motion', LAUNCH_FILES['robot_motion'])
            rospy.loginfo("Robot Motion Launch file started")
            return TriggerResponse(
                success=True,
                message="Launch file started successfully."
            )

        except Exception as e:
            rospy.logerr("Failed to start launch file: {}".format(e))
            return TriggerResponse(
                success=False,
                message="Failed to start launch file."
            )   
        
    def detection_enabler_callback(self, req):
        if 'detection' in self.processes:
            rospy.logwarn("A detection launch file is already running.")
            return TriggerResponse(
                success=False,
                message="Launch file is already running."
            )

        try:
            self.toggle_process(True, 'detection', LAUNCH_FILES['detection'])
            rospy.loginfo("Detection Launch file started")
            return TriggerResponse(
                success=True,
                message="Launch file started successfully."
            )

        except Exception as e:
            rospy.logerr("Failed to start launch file: {}".format(e))
            return TriggerResponse(
                success=False,
                message="Failed to start launch file."
            )

    def force_controller_enabler_callback(self, req):
        if 'force_controller' in self.processes:
            rospy.logwarn("A force controller launch file is already running.")
            return TriggerResponse(
                success=False,
                message="Launch file is already running."
            )

        try:
            self.toggle_process(True, 'force_controller', LAUNCH_FILES['force_controller'])
            rospy.loginfo("Force Controller Launch file started")
            return TriggerResponse(
                success=True,
                message="Launch file started successfully."
            )

        except Exception as e:
            rospy.logerr("Failed to start launch file: {}".format(e))
            return TriggerResponse(
                success=False,
                message="Failed to start launch file."
            )

    #################################################################################
    def toggle_process(self, state, name, launch_file):
        if state == True:
            process = RosLaunchProcess(self.uuid, PACKAGE_NAME, launch_file, self.status_queue)
            process.start()
            self.processes[name] = process
        else:
            if name in self.processes:
                self.processes[name].terminate()
                self.processes[name].join()
                del self.processes[name]



if __name__ == '__main__':
    server = ServerLauncher()
    rospy.spin()
            