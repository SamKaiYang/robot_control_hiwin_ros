
import rospy
import sys
import time
import threading
import argparse
import numpy as np
import math
import enum
import time
from std_msgs.msg import Int32MultiArray
import os
# yolo v4 import
from pushpin_mission.msg import ROI
#ROS message sent format
from std_msgs.msg import String
#Hiwin arm api class
from control_node import HiwinRobotInterface

DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0
positon = [0.0,36.8,11.35,-180,0,90]
Goal = [0.0,36.8,11.35,-180,0,90]
Current_pos = [0.0,0.0,0.0,0.0,0.0,0.0]

## pick down to box base [0 ,45.4, -40.7, -180,0,90]
## pick up to box above [0 ,45.4, -6.4, -180,0,90]
## place up target [39.4 ,1.09, -6.4, -180,0,90]
## place down target [39.4 ,1.09, -30.6, -180,0,90]

class Arm_cmd(enum.IntEnum):
    down_box_base = 1
    up_box_above = 2
    place_target = 3
    place_down_target = 4
    
 

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

class bounding_boxes():
    def __init__(self,probability,xmin,ymin,xmax,ymax,id_name,Class_name):
        self.probability = probability
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax
        self.id_name = str(id_name)
        self.Class_name = str(Class_name)

boxes = bounding_boxes(0,0,0,0,0,0,0)
# YOLO V4 input
def Yolo_callback(data):
    global obj_num
    #global probability,xmin,ymin,xmax,ymax,id_name,Class_name
    obj_num = len((data.object_name))
    if obj_num == 0:
        print("No Object Found!")
        switch_flag == 1
        print("change method to Realsense!")
    else:
        center_x = data.x
        center_y = data.y
        print("center_x:", center_x)
        print("center_y:", center_y)

## pick down to box base [0 ,45.4, -40.7, -180,0,90]
## pick up to box above [0 ,45.4, -6.4, -180,0,90]
## place up target [39.4 ,1.09, -6.4, -180,0,90]
## place down target [39.4 ,1.09, -30.6, -180,0,90]

##-------------strategy start ------------
def Mission_Trigger():
    if GetInfoFlag == True and GetKeyFlag == False and ExecuteFlag == False:
        GetInfo_Mission()
    if GetInfoFlag == False and GetKeyFlag == True and ExecuteFlag == False:
        GetKey_Mission()
    if GetInfoFlag == False and GetKeyFlag == False and ExecuteFlag == True:
        Execute_Mission()
def GetInfo_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag

    #Billiards_Calculation()

    GetInfoFlag = False
    GetKeyFlag = True
    ExecuteFlag = False

def GetKey_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionSerialKey

    Mission = Get_MissionType()
    MissionItem(Mission)
    MotionSerialKey = MotionKey
    GetInfoFlag = False
    GetKeyFlag = False
    ExecuteFlag = True

def Get_MissionType():
    global MissionType_Flag,CurrentMissionType
    for case in switch(MissionType_Flag): #傳送指令給socket選擇手臂動作
        if case(0):
            Type = MissionType.PushBall
            MissionType_Flag +=1
            break
        if case(1):
            Type = MissionType.Pushback
            MissionType_Flag -=1
            break
    CurrentMissionType = Type
    return Type
def MissionItem(ItemNo):
    global MotionKey
    Key_PushBallCommand = [\
        ArmMotionCommand.Arm_MoveToTargetUpside,\
        ArmMotionCommand.Arm_LineDown,\
        ArmMotionCommand.Arm_PushBall,\
        ArmMotionCommand.Arm_LineUp,\
        ArmMotionCommand.Arm_Stop,\
        ]
    Key_PushBackCommand = [\
        ArmMotionCommand.Arm_MoveVision,\
        ArmMotionCommand.Arm_Stop,\
        ArmMotionCommand.Arm_StopPush,\
        ]
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(MissionType.PushBall):
            MotionKey = Key_PushBallCommand
            break
        if case(MissionType.Pushback):
            MotionKey = Key_PushBackCommand
            break
    return MotionKey

def Execute_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionStep,MotionSerialKey,MissionEndFlag,CurrentMissionType
    global strategy_flag,Arm_state_flag
    if Arm_state_flag == 0 and strategy_flag == 1:
        strategy_flag = 0
        if MotionKey[MotionStep] == ArmMotionCommand.Arm_Stop:
            if MissionEndFlag == True:
                CurrentMissionType = MissionType.Mission_End
                GetInfoFlag = False
                GetKeyFlag = False
                ExecuteFlag = False
                print("Mission_End")
            elif CurrentMissionType == MissionType.PushBall:
                GetInfoFlag = False
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
                print("PushBall")
            else:
                GetInfoFlag = True
                GetKeyFlag = False
                ExecuteFlag = False
                MotionStep = 0
        else:
            MotionItem(MotionSerialKey[MotionStep])
            MotionStep += 1
def MotionItem(ItemNo):
    global angle_SubCue,SpeedValue,PushFlag,LinePtpFlag,MissionEndFlag
    SpeedValue = 5
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(ArmMotionCommand.Arm_Stop):
            MoveFlag = False
            print("Arm_Stop")
            break
        if case(ArmMotionCommand.Arm_StopPush):
            MoveFlag = False
            PushFlag = True #重新掃描物件
            print("Arm_StopPush")
            break
        if case(ArmMotionCommand.Arm_MoveToTargetUpside):
            pos.x = 10
            pos.y = 36.8
            pos.z = 11.35
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 10
            MoveFlag = True
            LinePtpFlag = False
            SpeedValue = 10
            print("Arm_MoveToTargetUpside")
            break
        if case(ArmMotionCommand.Arm_LineUp):
            pos.z = ObjAboveHeight
            MoveFlag = True
            LinePtpFlag = True
            SpeedValue = 5
            print("Arm_LineUp")
            break
        if case(ArmMotionCommand.Arm_LineDown):
            pos.z = PushBallHeight
            MoveFlag = True
            LinePtpFlag = True
            SpeedValue = 5
            print("Arm_LineDown")
            break
        if case(ArmMotionCommand.Arm_PushBall):
            pos.x = -10
            pos.y = 36.8
            pos.z = 11.35
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = -10
            SpeedValue = 10   ##待測試up
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_PushBall")
            break
        if case(ArmMotionCommand.Arm_MoveVision):
            pos.x = 0
            pos.y = 36.8
            pos.z = 11.35
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            SpeedValue = 10
            MoveFlag = True
            LinePtpFlag = False
            ##任務結束旗標
            MissionEndFlag = True
            print("Arm_MoveVision")
            break
        if case(ArmMotionCommand.Arm_MoveFowardDown):
            pos.x = 0
            pos.y = 36.8
            pos.z = 11.35
            pos.pitch = -90
            pos.roll = 0
            pos.yaw = 0
            MoveFlag = True
            LinePtpFlag = False
            print("Arm_MoveFowardDown")
            break
        if case(): # default, could also just omit condition or 'if True'
            print ("something else!")
            # No need to break here, it'll stop anyway
    if MoveFlag == True:
        if LinePtpFlag == False:
            print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            strategy_client_Speed_Mode(0)
            strategy_client_Arm_Mode(4,1,0,SpeedValue,2)#action,ra,grip,vel,both
            #strategy_client_Arm_Mode(2,1,0,SpeedValue,2)#action,ra,grip,vel,both
            #strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
        elif LinePtpFlag == True:
            #strategy_client_Arm_Mode(0,1,0,40,2)#action,ra,grip,vel,both
            print('x: ',pos.x,' y: ',pos.y,' z: ',pos.z,' pitch: ',pos.pitch,' roll: ',pos.roll,' yaw: ',pos.yaw)
            strategy_client_Speed_Mode(1)
            strategy_client_Arm_Mode(4,1,0,SpeedValue,2)#action,ra,grip,vel,both
            #strategy_client_Arm_Mode(3,1,0,SpeedValue,2)#action,ra,grip,vel,both
            #strategy_client_pos_move(pos.x,pos.y,pos.z,pos.pitch,pos.roll,pos.yaw)
    #action: ptp line
    #ra : abs rel
    #grip 夾爪
    #vel speed
    #both : Ctrl_Mode
##-------------strategy end ------------

def test_task():
    global ItemNo
    Arm_state = robot_ctr.get_robot_motion_state()
    if Arm_state == 1:
        if ItemNo==0:
            positon = [0.0,36.8,11.35,-180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 1
            print("task:0")
        elif ItemNo==1:
            positon = [0.0,46.8,11.35,-180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 2
            print("task:1")
        elif ItemNo==2:
            positon = [-10.0,26.8,11.35,-180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 3
            print("task:2")
        elif ItemNo==3:
            positon = [10.0,46.8,11.35,-180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            ItemNo = 4
            print("task:3")
        elif ItemNo==4:
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Go_home()
            ItemNo = 4
            print("task:4")

def myhook():
    print ("shutdown time!")

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

    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("obj_position",ROI,Yolo_callback)
    
    ## strategy trigger
    GetInfoFlag = True #Test no data
    try:
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            # set tool & base coor
            tool_coor = [0,0,180,0,0,0]
            base_coor = [0,0,0,0,0,0]
            robot_ctr.Set_base_number(1)
            base_result = robot_ctr.Define_base(1,base_coor)
            robot_ctr.Set_tool_number(1)
            tool_result = robot_ctr.Define_tool(1,tool_coor)

            robot_ctr.Set_operation_mode(1)
            robot_ctr.Set_override_ratio(10)
            robot_ctr.Set_acc_dec_ratio(100)

            # robot_ctr.Set_robot_output(2,False)
            # robot_ctr.Set_robot_output(3,False)
            # robot_ctr.Set_robot_output(4,False)
            robot_ctr.Set_digital_output(1,False)
            robot_ctr.Set_digital_output(2,False)
            robot_ctr.Set_digital_output(3,False)
        while(1):
            
            ### mission test 0914
            Mission_Trigger()
            if CurrentMissionType == MissionType.Mission_End:
                rospy.on_shutdown(myhook)
            ### mission test 0914


            #test_task()

            #print("boxes:",boxes)
            #robot_ctr.Set_digital_input(2,1)

            robot_outputs_state = robot_ctr.Get_current_robot_outputs()
            robot_inputs_state = robot_ctr.Get_current_robot_inputs()
            digital_output_state = robot_ctr.Get_current_digital_outputs()
            
            #print("robot outputs state:",robot_outputs_state)
            #print("robot inputs state:",robot_inputs_state)

            #pose = robot_ctr.Get_current_position()
            #print("pose:",pose)
        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass

