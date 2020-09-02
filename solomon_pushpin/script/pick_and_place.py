
import rospy
import sys
import time
import threading
import argparse
import numpy as np
import os
from std_msgs.msg import String
import rospy
from rt605_arm_control_api import HiwinRobotInterface

DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0
positon = [0.0,36.8,11.35,-90,0,0]
Goal = [0.0,36.8,11.35,-90,0,0]
##-----------switch define------------##
class switch(object):
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration

    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: # changed for v1.5, see below
            self.fall = True
            return True
        else:
            return False
class pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 36.8
        self.z = 11.35
        self.pitch = -90
        self.roll = 0
        self.yaw = 0
def test_task():
    global ItemNo
    #robot_ctr.Set_motor_state(state)
    if robot_ctr.is_in_idle == True:
        for case in switch(ItemNo):
            if case(0):
                robot_ctr.Step_AbsPTPCmd(pos)
                ItemNo = 1
            if case(1):
                pos.x = pos.x + 1
                pos.y = pos.y + 1
                robot_ctr.Step_AbsPTPCmd(pos)
                ItemNo = 2
            if case(2):
                pos.x = pos.x + 1
                pos.y = pos.y + 1
                robot_ctr.Step_AbsPTPCmd(pos)
                ItemNo = 3
            if case(3):
                pos.x = pos.x + 1
                pos.y = pos.y + 1
                robot_ctr.Step_AbsPTPCmd(pos)
                ItemNo = 4
            if case(4):
                robot_ctr.Go_home()
                ItemNo = 4


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser("Driver Node")
    arg_parser.add_argument("--robot_ip", help="IP addr of the robot",
                            type=str)
    arg_parser.add_argument("--robot_name", help="Name of the robot", type=str)
    arg_parser.add_argument("--control_mode", help="Default is 1, set it to 0 if you do not want to control the robot, but only to monitor its state.",
                            type=bool, default=1, required=False)
    arg_parser.add_argument("--log_level", help="Logging level: INFO, DEBUG",
                            type=str, default="INFO", required=False)
    arg_parser.add_argument("__name")
    arg_parser.add_argument("__log")
    args = arg_parser.parse_args()

    # Extract the necessary arguments
    robot_ip = args.robot_ip
    robot_name = args.robot_name
    control_mode = int(args.control_mode)
    if args.log_level == "DEBUG":
        log_level = rospy.DEBUG
    elif args.log_level == "ERROR":
        log_level = rospy.ERROR
    else:
        log_level = rospy.INFO
    
    # Start the ROS node
    rospy.init_node('hiwin_robot_sdk_'+robot_name,
                    log_level=log_level,
                    disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logwarn("use_sim_time is set!!!")

    robot_ctr = HiwinRobotInterface(robot_ip=robot_ip, connection_level=control_mode,name=robot_name)
    robot_ctr.connect()
    #robot_mtr = HiwinRobotInterface(robot_ip=robot_ip, connection_level=1,name=robot_name)
    try:
        #rospy.on_shutdown(myhook)
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Set_override_ratio(50)
        while(1):
            #time.sleep(1)
            aa = robot_ctr.get_robot_motion_state()
            print("get_robot_motion_state:",aa)
            if aa == 1:
                if ItemNo==0:
                    robot_ctr.Step_AbsPTPCmd(positon)
                    ItemNo = 1
                    print("task:0")
                elif ItemNo==1:
                    positon = [0.0,46.8,11.35,0,-90,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    ItemNo = 2
                    print("task:1")
                elif ItemNo==2:
                    positon = [-10.0,26.8,11.35,-90,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    ItemNo = 3
                    print("task:2")
                elif ItemNo==3:
                    positon = [10.0,46.8,11.35,-90,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    ItemNo = 4
                    print("task:3")
                elif ItemNo==4:
                    robot_ctr.Go_home()
                    ItemNo = 4
                    print("task:4")
            
        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass

