#!/usr/bin/python
# -*- coding: UTF-8 -*-
import rospy
import sys
import time
import numpy as np
import os
import datetime
from ctypes import *
import rospy
from enum import Enum
import threading
from control_node.msg import robot_info

# Init the path to the Hiwin Robot's SDK .dll file
CURRENT_FILE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
PARENT_DIRECTORY = os.path.dirname(CURRENT_FILE_DIRECTORY)
# The .dll file is contained in include\hiwin_robot_sdk\
HRSDK_DLL_PATH = os.path.join(PARENT_DIRECTORY, "include", "hiwin_robot_sdk",
                              "HRSDK.dll")
class pos():
    def __init__(self, x, y, z, pitch, roll, yaw):
        self.x = 0
        self.y = 36.8
        self.z = 11.35
        self.pitch = -90
        self.roll = 0
        self.yaw = 0
def callback_function(cmd, rlt, msg, len):
    #print(cmd)
    pass
##ABS pos mm2cm
def AbsPostoGoal(pos):
    pos[0] = pos[0]/0.1
    pos[1] = pos[1]/0.1
    pos[2] = pos[2]/0.1
    return pos
##relate pos 
def RelPosConvertGoal(pos):
    if pos[0] != 0:
        pos[0] = pos[0]/0.1
    else: 
        pos[0] =0
    if pos[1] !=0:
        pos[1] = pos[1]/0.1
    else:
        pos[1] =0
    if pos[2] !=0:
        pos[2] = pos[2]/0.1
    else:
        pos[2] =0
    return pos
def ctype_convert(target):
    target_convert = (c_double * len(target))()
    for convert_n, target in enumerate(target):
        target_convert[convert_n] = c_double(target)
    return target_convert

class HiwinRobotInterface(object):
    """Class used as bridge python-CPP and SDK."""
    # The value of the robot state
    IDLE_MOTION_STATE = 1
    RUNNING_MOTION_STATE = 2
    
    def __init__(self, robot_ip, connection_level, name=""):
        # type: (str, int, str, str) -> None
        """Hiwin Robot SDK Initialization"""
        # Initialize the variables
        self.ip = robot_ip
        self.level = connection_level
        self.robot_id = -1
        self.name = name
        self.CurrGoal = [0.0,36.8,11.35,-180,0,90]
        self.positon = [0.0,36.8,11.35,-180,0,90]
        self.Goal = [0.0,36.8,11.35,-180,0,90]
        # Load the SDK
        # Make sure the SKL library absolute file contains the file

        assert os.path.exists(HRSDK_DLL_PATH), \
            "HRSDK not found. Given path: {path}".format(path=HRSDK_DLL_PATH)
        self.HRSDKLib = cdll.LoadLibrary(HRSDK_DLL_PATH)
        # try:
        #     self.HRSDKLib.set_log_level(c_int(3))
        # except AttributeError:
        #     pass

        # Get the callback function
        callback_type = CFUNCTYPE(None, c_uint16, c_uint16,
                                  POINTER(c_uint16), c_int)
        self.callback = callback_type(callback_function)
        self.reconnecting = False  # Used to know if we are trying to reconnect

    #     self.__pub_threads = threading.Thread(target=self.__pub_robot_info)
    #     self.__robot_info_pub = rospy.Publisher(
    #         'robot/curr_info',
    #         robot_info,
    #         queue_size=1
    #     )
    #     print("A")
    #     self.__pub_threads.setDaemon(True)
    #     self.__pub_threads.start()

    # def __pub_robot_info(self):
    #     rate = rospy.Rate(10)
    #     while not rospy.is_shutdown():
    #         try:
    #             msg = robot_info()
    #             msg.curr_pose = self.Get_current_position()
    #             _, msg.tool_coor = self.Get_tool_data()
    #             # _, msg.base_coor = self.Get_base_data()
    #             self.__robot_info_pub.publish(msg)
    #             rate.sleep()
    #         except KeyboardInterrupt:
    #             break



    def connect(self):  # type: () -> bool
        """Connect to the Hiwin robot

        :param
                ip   : Computer connect to robot (str)
                level: Connection level (int)
        :return
            success: True if connection has succeeded, False otherwise (bool)
        """
        self.robot_id = self.HRSDKLib.open_connection(self.ip, c_int(self.level),
                                                    self.callback)
        if self.is_connected():
            success = True
            if self.level == 1:
                # Initialize some parametes
                #   set operation mode to "Auto"
                self.HRSDKLib.set_operation_mode(c_int(self.robot_id),
                                                c_int(1))
                self.HRSDKLib.set_override_ratio(c_int(self.robot_id),
                                                c_int(10))
            rospy.loginfo("HIWIN Robot '{}' successfully connected.".format(self.name))
        else:
            success = False
        return success
    def reconnect(self, trials=5, sec_between_trials=2.0):
        # type: (int, float) -> bool
        """Try to reconnect to the robot. The ip and connection level for the
        connection are taken from the ones given during __init__().

        :param trials: Number of time to try to reconnect (int)
        :param sec_between_trials: seconds to sleep between each trial (float)

        :return success: True if correctly connected, False otherwise
        """
        # Get the connection level
        connection_level = self.get_connection_level()
        # If robot is already connected with the correct level, nothing to do.
        if connection_level == self.level:
            success = True
            return success

        # If the robot is already reconnecting, do nothing
        if self.reconnecting:
            return False
        self.reconnecting = True

        # Try to reconnect to the robot
        for trial in xrange(trials):
            rospy.loginfo('Reconnecting to HIWIN robot "{robot_name}": '
                          'trial #{trial_num}.'.format(robot_name=self.name,
                                                       trial_num=trial+1))
            # Connect to the robot
            success = self.connect()
            if success:
                rospy.loginfo('Successfully reconnected with the robot!')
                self.reconnecting = False
                return success
            else:
                self.close()
                # Retry after a while
                time.sleep(sec_between_trials)
        rospy.logwarn('Could not reconnect to robot "{robot_name}"! '
                      'Total trials: {trials_num}'
                      .format(robot_name=self.name, trials_num=trials))
        self.reconnecting = False
        return False

    def close(self):
        # type: () -> bool
        """Disconnect to robot

        :return
            Success: True if successfully disconnected, False otherwise
        """
        error_id = self.HRSDKLib.disconnect(c_int(self.robot_id))
        #close_connection => disconnect 2.2.9
        # If correctly disconnected error_id is equal to 0
        if error_id == 0:
            return True
        else:
            return False
    def is_connected(self):
        # type: () -> bool
        """Function to know if the robot is currently connected.

        :return
            is_connected: True if the robot is connected, False otherwise
        """
        connection_level = self.get_connection_level()
        # If connection_level is -1 it means that the robot is disconnected
        is_connected = (connection_level == self.level)
        return is_connected

    def is_in_state(self, joints_states, angle_threshold=0.01):
        # type: (list[float], float) -> bool
        """Check if the robot is in the given state (or close enough).

        The robot is in the state if all the angles are the same as the given
        joints states (allowing a optional angle_threshold)

        :param joints_states: list of joints angles expressed in radians
        :param angle_threshold: value (in radians) over which two angles are
                                considered different one to the other.
        """
        success, current_joints_states = self.get_current_joints()

        # For each joint of the robot
        for current_joint_state, joint_state in zip(
                current_joints_states, joints_states):
            # Check if the current value is the same as the input one
            if abs(current_joint_state-joint_state) >\
                    angle_threshold:
                # One of the joints is not in the state input
                return False  # The robot is not in the given state
        # All the joints of the
        return True
# arm state whether idle
    def is_in_idle(self):
        # type: () -> bool
        """Tells whether the robot is in IDLE or not."""
        robot_motion_state = self.get_robot_motion_state()
        is_in_idle_state = (robot_motion_state == self.IDLE_MOTION_STATE)
        return is_in_idle_state
# arm state whether busy
    def is_running(self):
        # type: () -> bool
        """Tells whether the robot is running (moving) or not."""
        robot_motion_state = self.get_robot_motion_state()
        is_running = (robot_motion_state == self.RUNNING_MOTION_STATE)
        return is_running

    def get_hrsdk_version(self):
        # type: () -> (int, str)
        """Get HRSDK version

        :return
            error_id:
                Success :0
                Fail    :else
            version   : HRSDK version (string)
        """
        version = create_string_buffer(15)
        error_id = self.HRSDKLib.get_HRSDK_version(version)
        return error_id, version.value.decode("utf-8")

    def get_connection_level(self):
        # type: () -> int
        """Get user connect level to the robot

        :return
            Connection level:
                Operator :0
                Expert   :1
        """
        # TODO: Check if the robot is connected first
        connection_level = self.HRSDKLib.get_connection_level(
            c_int(self.robot_id))
        return connection_level

    def set_connection_level(self, level):
        # type: (int) -> bool
        """Get user connect level

        :parameter
            level:
                Operator :0
                Expert   :1
        :return
            bool:
                True: success
                False: failure
        """
        result = self.HRSDKLib.set_control_level(c_int(self.robot_id),
                                                 c_int(level))
        if result == level:
            return True
        elif result != level:
            return False
# arm state return:
    # 1:idle
    # 2:motin
    # 3:stop
    # 4:delay
    # 5:commend stay
    # fail: alarm code -1
    def get_robot_motion_state(self):
        return self.HRSDKLib.get_motion_state(self.robot_id)

##  PtP motion ABS 
    def Step_AbsPTPCmd(self, Pos, mode=0):
        Pos_abs = AbsPostoGoal(Pos)
        Pos_ctype = ctype_convert(Pos_abs)
        self.HRSDKLib.ptp_pos(c_int(self.robot_id), c_int(mode),Pos_ctype)

##  Line motion ABS
    def Step_AbsLine_PosCmd(self, Pos, mode=0, smooth_value=0):
        Pos_abs = AbsPostoGoal(Pos)
        Pos_ctype = ctype_convert(Pos_abs)
        self.HRSDKLib.lin_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),Pos_ctype)

##  PtP motion relate
    def Step_RelPTPCmd(self, Pos_rel, mode=0):
        Pos_rel = RelPosConvertGoal(Pos_rel)
        Pos_ctype = ctype_convert(Pos_rel)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode),Pos_ctype)

##  Line motion relate
    def Step_RelLineCmd(self, Pos_rel, mode=0, smooth_value=0):
        Pos_rel = RelPosConvertGoal(Pos_rel)
        Pos_ctype = ctype_convert(Pos_rel)
        self.HRSDKLib.lin_rel_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),Pos_ctype)

    def Stop_motion(self):
        """Stop the motion of the robot."""
        self.HRSDKLib.motion_abort(self.robot_id)
    def Continue_motion(self):
        """continue the motion of the robot."""
        self.HRSDKLib.motion_continue(self.robot_id)
    def Hold_motion(self):
        """Hold the motion of the robot."""
        self.HRSDKLib.motion_hold(self.robot_id)
    def Delay_motion(self,delay):
        """Delay the motion of the robot."""
        self.HRSDKLib.motion_delay(self.robot_id,c_int(delay))

    def Get_current_position(self):
        Current_Pos = (c_double * 6)()
        result = self.HRSDKLib.get_current_position(c_int(self.robot_id),Current_Pos)
        #Current_Pos = float(Current_Pos)
        value = [float(value) for value in (Current_Pos)]
        value[0:3] = [ele/10 for ele in value[0:3]]
        return value

#------set system variable
    #set all arm speed
    def Set_override_ratio(self,Speed):
        self.HRSDKLib.set_override_ratio(c_int(self.robot_id), c_int(Speed))

    #get all arm speed
    def Get_override_ratio(self):
        override_ratio = self.HRSDKLib.get_override_ratio(c_int(self.robot_id))
        return override_ratio

    #set all arm acceleration
    def Set_acc_dec_ratio(self,acc): 
        self.HRSDKLib.set_acc_dec_ratio(c_int(self.robot_id), c_int(acc))

    #get all arm acceleration
        #only Auto mode can set the ratio of acceleration/deceleration
    def Get_acc_dec_ratio(self): 
        acc_ratio = self.HRSDKLib.get_acc_dec_ratio(c_int(self.robot_id))
        return acc_ratio

    #set all arm acceleration time
    def Set_acc_time(self,value):
        self.HRSDKLib.set_acc_time(c_int(self.robot_id), c_int(value))

    #get all arm acceleration time
    def Get_acc_time(self):
        acc_time = self.HRSDKLib.get_acc_time(c_int(self.robot_id))
        return acc_time

    #set arm PTP motion speed
    def Set_ptp_speed(self,Speed):
        self.HRSDKLib.set_ptp_speed(c_int(self.robot_id), c_int(Speed))

    #get arm PTP motion speed
    def Get_ptp_speed(self):
        return self.HRSDKLib.get_ptp_speed(c_int(self.robot_id))

    #set arm LINE motion speed
    def Set_lin_speed(self,Speed):
        return self.HRSDKLib.set_lin_speed(c_int(self.robot_id), c_double(Speed))

    #get arm LINE motion speed
    def Get_lin_speed(self):
        return self.HRSDKLib.get_lin_speed(c_int(self.robot_id))

    # arm back home motion 
        #only Manual mode can set
    def Go_home(self):
        self.HRSDKLib.jog_home(c_int(self.robot_id))

    #jog stop
    def Jog_stop(self):
        self.HRSDKLib.jog_stop(c_int(self.robot_id))

    # set robot base number
    def Set_base_number(self,basenum):
        self.HRSDKLib.set_base_number(c_int(self.robot_id),c_int(basenum))

    def Get_base_number(self):
        return self.HRSDKLib.get_base_number(c_int(self.robot_id))

    # set robot base 
    def Define_base(self,basenum,Coor):
        Coor_ctype = ctype_convert(Coor)
        result = self.HRSDKLib.define_base(c_int(self.robot_id),c_int(basenum),Coor_ctype)
        return result

    # get robot base
    def Get_base_data(self):
        Coor = (c_double * 6)()
        basenum = self.Get_base_number()
        result = self.HRSDKLib.get_base_data(c_int(self.robot_id),c_int(basenum),Coor)
        value = [float(value) for value in (Coor)]
        value[0:3] = [ele/10 for ele in value[0:3]]
        return result == 0, value

    # set tool number
    def Set_tool_number(self,toolnum):
        self.HRSDKLib.set_tool_number(c_int(self.robot_id),c_int(toolnum))

    # get tool number
    def Get_tool_number(self):
        return self.HRSDKLib.get_tool_number(c_int(self.robot_id))

    def Define_tool(self,toolnum,Coor):
        Coor_ctype = ctype_convert(Coor)
        result = self.HRSDKLib.define_tool(c_int(self.robot_id),c_int(toolnum),Coor_ctype)
        return result

    def Get_tool_data(self):
        Coor = (c_double * 6)()
        toolnum = self.Get_tool_number()
        result = self.HRSDKLib.get_tool_data(c_int(self.robot_id),c_int(toolnum),Coor)
        value = [float(value) for value in (Coor)]
        value[0:3] = [ele/10 for ele in value[0:3]]
        return result == 0, value

#  # Servo on: 1   Servo off: 0
    def Set_motor_state(self, state):
        self.HRSDKLib.set_motor_state(c_int(self.robot_id),c_int(state))

# get motor state
    def Get_motor_state(self):
        return self.HRSDKLib.get_motor_state(self.robot_id)

#  Manual mode: 0  Auto mode: 1
    def Set_operation_mode(self,mode):
        self.HRSDKLib.set_operation_mode(c_int(self.robot_id),c_int(mode))

    def Get_operation_mode(self):
        return self.HRSDKLib.get_operation_mode(self.robot_id)

    def Clear_alarm(self):
        self.HRSDKLib.clear_alarm(c_int(self.robot_id))

    # def Get_alarm_code(self):
    #     alarm_code = np.zeros(20,dtype=np.c_uint64)
    #     #alarm_code = np.uint64(alarm_code)
    #     #alarm_code = (c_uint64 * 20)()
    #     count = 20
    #     result = self.HRSDKLib.get_alarm_code(c_int(self.robot_id),c_int(count),alarm_code)
    #     return result == 0, [float(value) for value in (alarm_code)]
# I/O control 
    def Get_current_digital_inputs(self):
        # type: () -> (list[int])
        """Get Robot current digital inputs.

        :returns
            inputs: list of the value of the digital inputs
            (1 if on 0 if off)
        """
        # If the robot is not connected, try reconnecting
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to get "
                              "the digital inputs")
                return [-1 for _ in range(48)]
        inputs = []
        for i in range(1, 49):
            inputs.append(self.HRSDKLib.get_digital_input(c_int(self.robot_id),
                                                          c_int(i)))
        return inputs

    def Get_current_digital_outputs(self):
        # type: () -> (list[int])
        """Get Robot current digital outputs.

        :returns
            outputs: list of the value of the digital outputs
            (1 if on 0 if off)
        """
        # If the robot is not connected, try reconnecting
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to get "
                              "the digital outputs")
                return [-1 for _ in range(48)]
        outputs = []
        for i in range(1, 49):
            outputs.append(self.HRSDKLib.get_digital_output(c_int(self.robot_id),
                                                           c_int(i)))
        return outputs

    def Set_digital_output(self,index,value):
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to set "
                              "the digital outputs")
        self.HRSDKLib.set_digital_output(c_int(self.robot_id), c_int(index),c_bool(value))

    
    def Set_robot_output(self,index,value):
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to set "
                              "the digital outputs")
        self.HRSDKLib.set_robot_output(c_int(self.robot_id), c_int(index),c_bool(value))

    def Get_current_robot_outputs(self):
        # type: () -> (list[int])
        """Get Robot current robot outputs.

        :returns
            outputs: list of the value of the robot outputs
            (1 if on 0 if off)
        """
        # If the robot is not connected, try reconnecting
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to get "
                              "the robot outputs")
                return [-1 for _ in range(8)]
        outputs = []
        for i in range(1, 9):
            outputs.append(self.HRSDKLib.get_robot_output(c_int(self.robot_id),
                                                           c_int(i)))
        return outputs
    def Get_current_robot_inputs(self):
        # type: () -> (list[int])
        """Get Robot current digital inputs.

        :returns
            inputs: list of the value of the digital inputs
            (1 if on 0 if off)
        """
        # If the robot is not connected, try reconnecting
        if not self.is_connected():
            successfully_reconnected = self.reconnect()
            if not successfully_reconnected:
                rospy.logwarn("Robot disconnected, it was not possible to get "
                              "the digital inputs")
                return [-1 for _ in range(8)]
        inputs = []
        for i in range(1, 9):
            inputs.append(self.HRSDKLib.get_robot_input(c_int(self.robot_id),
                                                          c_int(i)))
        return inputs