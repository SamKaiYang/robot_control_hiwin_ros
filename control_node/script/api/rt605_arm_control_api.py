import rospy
import sys
import time
import numpy as np
import os
import datetime

from ctypes import *
import rospy
from enum import Enum

# Init the path to the Hiwin Robot's SDK .dll file
CURRENT_FILE_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
PARENT_DIRECTORY = os.path.dirname(CURRENT_FILE_DIRECTORY)
# The .dll file is contained in include\hiwin_robot_sdk\
HRSDK_DLL_PATH = os.path.join(PARENT_DIRECTORY, "include", "hiwin_robot_sdk",
                              "HRSDK.dll")
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
        self.Goal = []
        self.CurrGoal = []
        # Load the SDK
        # Make sure the SKL library absolute file contains the file
        assert os.path.exists(HRSDK_DLL_PATH), \
            "HRSDK not found. Given path: {path}".format(path=HRSDK_DLL_PATH)
        self.HRSDKLib = cdll.LoadLibrary(HRSDK_DLL_PATH)
        try:
            self.HRSDKLib.set_log_level(c_int(3))
        except AttributeError:
            pass
        # Get the callback function
        callback_type = CFUNCTYPE(None, c_uint16, c_uint16,
                                  POINTER(c_uint16), c_int)
        self.callback = callback_type(callback_function)
        self.reconnecting = False  # Used to know if we are trying to reconnect
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
                                                c_int(100))
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
        error_id = self.HRSDKLib.close_connection(c_int(self.robot_id))

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
# 判斷手臂狀態是否為閒置
    def is_in_idle(self):
        # type: () -> bool
        """Tells whether the robot is in IDLE or not."""
        robot_motion_state = self.get_robot_motion_state()
        is_in_idle_state = (robot_motion_state == self.IDLE_MOTION_STATE)
        return is_in_idle_state
# 判斷手臂狀態是否為忙碌
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
# 判斷手臂狀態 回傳值如下
    # 1:閒置狀態
    # 2:運動狀態
    # 3:暫停狀態
    # 4:延遲狀態
    # 5:命令等待狀態
    # 失敗: 錯誤碼 -1
    def get_robot_motion_state(self):
        return self.HRSDKLib.get_motion_state(self.robot_id)

##ABS pos convert
    def AbsPostoGoal(self, pos):
	    self.Goal[0] = pos.x
    	self.Goal[0] /= 0.1
	    self.Goal[1] = pos.y
        self.Goal[1] /= 0.1
	    self.Goal[2] = pos.z
        self.Goal[2] /= 0.1
	    #arm pose(-90 ,0 ,0)to(0 ,180 ,90) 
	    self.Goal[4] = pos.pitch
        self.Goal[4] +=  90 
        self.Goal[4]=goal[4]*(-1)
	    self.Goal[3] = pos.roll
        self.Goal[3] += 180
	    self.Goal[5] = pos.yaw
        self.Goal[5] +=  90
##  PtP motion ABS
    def Step_AbsPTPCmd(self, Pos, mode=0):
        AbsPostoGoal(pos)
        self.HRSDKLib.ptp_pos(c_int(self.robot_id), c_int(mode),self.Goal)

    def Step_AbsPTP_PosCmd(self, Pos, mode=0): ##純動PTP x,y,z
        self.HRSDKLib.get_current_position(c_int(self.robot_id), self.CurrGoal)
        self.Goal[3] = CurrGoal[3]
        self.Goal[4] = CurrGoal[4]
        self.Goal[5] = CurrGoal[5]
        self.HRSDKLib.ptp_pos(c_int(self.robot_id), c_int(mode),self.Goal)

    def Step_AbsPTP_EulerCmd(self, Pos, mode=0): ##純動line roll,pitch,yaw
        self.HRSDKLib.get_current_position(c_int(self.robot_id), self.CurrGoal)
        self.Goal[0] = CurrGoal[0]
        self.Goal[1] = CurrGoal[1]
        self.Goal[2] = CurrGoal[2]
        self.HRSDKLib.ptp_pos(c_int(self.robot_id), c_int(mode),self.Goal)
##  Line motion ABS
    def Step_AbsLine_PosCmd(self, Pos, mode=0, smooth_value=0):
        AbsPostoGoal(pos)
        self.HRSDKLib.lin_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),self.Goal)

    def Step_AbsLine_PosCmd(self, Pos, mode=0, smooth_value=0): ##純動line x,y,z
        self.HRSDKLib.get_current_position(c_int(self.robot_id), self.CurrGoal)
        self.Goal[3] = CurrGoal[3]
        self.Goal[4] = CurrGoal[4]
        self.Goal[5] = CurrGoal[5]
        self.HRSDKLib.lin_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value), self.Goal)

    def Step_AbsLine_EulerCmd(self, Pos, mode=0, smooth_value=0): ##純動line roll,pitch,yaw
        self.HRSDKLib.get_current_position(c_int(self.robot_id), self.CurrGoal)
        self.Goal[0] = CurrGoal[0]
        self.Goal[1] = CurrGoal[1]
        self.Goal[2] = CurrGoal[2]
        self.HRSDKLib.lin_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value), self.Goal)
##relate pos convert
    def RelPosConvertGoal(self, pos):
        if pos.x !=0:
		    self.Goal[0]	= pos.x
            self.Goal[0] /= 0.1
	    else:
		    self.Goal[0] =0
	    if pos.y !=0:
	    	self.Goal[1] = pos.y	
            self.Goal[1] /= 0.1
	    else:
		    self.Goal[1] =0
        if pos.z !=0:
            self.Goal[2] = pos.z
            self.Goal[2] /= 0.1
        else:
            self.Goal[2] =0
        #arm pose(-180, 0, 90)to(-90, 0 , 0)
        if pos.roll !=0
            self.Goal[3] = pos.roll
        else:
            self.Goal[3] =0
        if pos.pitch !=0:
            self.Goal[4] = pos.pitch
        else:
            self.Goal[4] =0
        if pos.yaw !=0:
            self.Goal[5] = pos.yaw
        else:
            self.Goal[5] =0
##  PtP motion relate
    def Step_RelPTPCmd(self, Pos, mode=0):
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode),self.Goal)
    def Step_RelPTP_PosCmd(self, Pos, mode=0):
        Pos.pitch = 0
        Pos.roll = 0
        Pos.yaw = 0
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode),self.Goal)
    def Step_RelPTP_EulerCmd(self, Pos, mode=0):
        Pos.x = 0
        Pos.y = 0
        Pos.z = 0
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode),self.Goal)
##  Line motion relate
    def Step_RelLineCmd(self, Pos, mode=0, smooth_value=0):
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),self.Goal)
    def Step_RelLine_PosCmd(self, Pos, mode=0, smooth_value=0):
        Pos.pitch = 0
        Pos.roll = 0
        Pos.yaw = 0
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),self.Goal)
    def Step_RelLine_EulerCmd(self, Pos, mode=0, smooth_value=0):
        Pos.x = 0
        Pos.y = 0
        Pos.z = 0
        RelPosConvertGoal(pos)
        self.HRSDKLib.ptp_rel_pos(c_int(self.robot_id), c_int(mode), c_int(smooth_value),self.Goal)

    def Stop_motion(self):
        """Stop the motion of the robot."""
        self.HRSDKLib.motion_abort(self.robot_id)
#------設定系統變數 
    #設定全部手臂動作速度
    def Set_override_ratio(self,Speed):
        self.HRSDKLib.set_override_ratio(c_int(self.robot_id), c_int(Speed))
    #讀取全部手臂動作速度
    def Get_override_ratio(self):
        override_ratio = self.HRSDKLib.get_override_ratio(c_int(self.robot_id))
        return override_ratio
    #設定全部手臂動作加速度
    def Set_acc_dec_ratio(self,acc): 
        self.HRSDKLib.set_acc_dec_ratio(c_int(self.robot_id), c_int(acc))
    #讀取全部手臂動作加速度
        #自動模式才可以設定加/減速度的比例
    def Get_acc_dec_ratio(self): 
        acc_ratio = self.HRSDKLib.get_acc_dec_ratio(c_int(self.robot_id))
        return acc_ratio
    #設定全部手臂動作加速度時間
    def Set_acc_time(self,value):
        self.HRSDKLib.set_acc_time(c_int(self.robot_id), c_int(value))
    #讀取全部手臂動作加速度時間
    def Get_acc_time(self):
        acc_time = self.HRSDKLib.get_acc_time(c_int(self.robot_id))
        return acc_time
    #設定手臂動作 PTP 運動速度
    def Set_ptp_speed(self,Speed):
        self.HRSDKLib.set_ptp_speed(c_int(self.robot_id), c_int(Speed))
    #讀取手臂動作 PTP 運動速度
    def Get_ptp_speed(self):
        self.HRSDKLib.get_ptp_speed(c_int(self.robot_id))
    #設定手臂動作 LINE 運動速度
    def Set_lin_speed(self,Speed):
        self.HRSDKLib.set_lin_speed(c_int(self.robot_id), c_int(Speed))
    #讀取手臂動作 LINE 運動速度
    def Get_lin_speed(self):
        self.HRSDKLib.get_lin_speed(c_int(self.robot_id))
    # 手臂回home
    def Go_home(self):
        self.HRSDKLib.jog_home(c_int(self.robot_id))
    #軸動停止
    def Jog_stop(self):
        self.HRSDKLib.jog_stop(c_int(self.robot_id))
    # 設定工具號碼
    def Set_tool_number(self,num):
        self.HRSDKLib.set_tool_number(c_int(self.robot_id),c_int(num))
    # 讀取工具號碼
    def Get_tool_number(self,num):
        self.HRSDKLib.get_tool_number(c_int(self.robot_id))
    # 定義工具之工具座標
    # def Define_tool(self,toolnum,coor):
    #     self.HRSDKLib.define_tool(c_int(self.robot_id),c_int(toolnum),c_int(coor))
#HRSDK_API int __stdcall define_tool(HROBOT robot, int toolNum, double *coor);
#HRSDK_API int __stdcall get_tool_data(HROBOT robot, int num, double* coor);

# void Set_ToolCoordinate(int toolNum, double* coor)
# 		 {
# 			 cout << "define tool: " << define_tool( Arm->RobotID, toolNum, coor ) << endl;
# 			 cout << "set_tool_number: " << set_tool_number( Arm->RobotID, toolNum ) << endl;
# 		 }
# private: System::Void btn_ToolClass_Click(System::Object^ sender, System::EventArgs^ e) 
# 		{
# 			int toolNum = 2;
# 			iMotion = true;  //讓外部電腦可以使用緊急停止
# 			iCurret_Pos = true;  //讓外部電腦可存取手臂當前位置
# 			lab_Tool->Text = "Class";
# 			double coor[6] = { 0, 0, TOOLZ_CLASS, 0, 0, 0 };
# 			Set_ToolCoordinate(TOOLNUM_CLASS, coor);
# 		}

# 設定馬達激磁 # Servo on: 1   Servo off: 0
    def Set_motor_state(self, state):
        self.HRSDKLib.set_motor_state(c_int(self.robot_id),c_int(state))
# 讀取馬達狀態
    def Get_motor_state(self):
        self.HRSDKLib.get_motor_state(c_int(self.robot_id))
# 設定操作模式 #  手動模式: 0  自動模式: 1
    def Set_operation_mode(self,mode):
        self.HRSDKLib.set_operation_mode(c_int(self.robot_id),c_int(mode))
# 讀取目前操作模式
    def Get_operation_mode(self):
        self.HRSDKLib.get_operation_mode(c_int(self.robot_id))
# 清除錯誤代碼警告
    def Clear_alarm(self):
        self.HRSDKLib.clear_alarm(c_int(self.robot_id))