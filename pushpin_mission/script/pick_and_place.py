
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

##-----Mission 參數
GetInfoFlag = False
ExecuteFlag = False
GetKeyFlag = False
MotionSerialKey = []
MissionType_Flag = 0
MotionStep = 0

arm_move_times = 1


class Arm_cmd(enum.IntEnum):
    MoveToObj_Pick = 1
    MoveToTarget_Place = 2
    Absort_ON = 3
    Absort_OFF = 4
    MoveToObj_PickUp = 5
    MoveToTarget_PlaceUp = 6
    Arm_Stop = 7
    # down_box_base = 3
    # up_box_above = 4
    # place_target = 5
    # place_down_target = 6
class MissionType(enum.IntEnum):
    Get_Img = 0
    Pick = 1
    Place = 2
    Mission_End = 3
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
##------------------yolo_v3 stra.py-----------------
# def get_obj_info_cb(data):
#     global object_name,min_xy,max_xy,obj_num,score,pic_times,ball_mid,SpecifyBall_mid,CueBalll_mid
#     global GetInfoFlag
#     print("\n========== Detected object number = " + str(len(data.ROI_list)) + " ========== ")
#     for obj_num in range(len(data.ROI_list)):
#         object_name = data.ROI_list[obj_num].object_name
#         score       = data.ROI_list[obj_num].score
#         min_xy = np.array([data.ROI_list[obj_num].min_x, data.ROI_list[obj_num].min_y])
#         max_xy = np.array([data.ROI_list[obj_num].Max_x, data.ROI_list[obj_num].Max_y])

#         if(obj_num!=0):
#             print("\n")
#         print("----- object_" + str(obj_num) + " ----- ")
#         print("object_name = " + str(object_name))
#         print("score = " + str(score))
#         print("min_xy = [ " +  str(min_xy) +  " ]" )
#         print("max_xy = [ " +  str(max_xy) +  " ]" )

#         print("mid_xy = ["+str((min_xy+max_xy)/2)+"]")
#     ##-- yolov3 info
#     ## for 取 10張 信心值超過70%
#     ## 取出位置取平均

#         if  str(len(data.ROI_list)) == 2 and score >= 70:
#             pic_times+=1
#             if object_name == "Specify":
#                 SpecifyBall_mid = np.array([SpecifyBall_mid + (min_xy+max_xy)/2])
#             if object_name == "Nine":
#                 CueBalll_mid = np.array([CueBalll_mid[1] +  (min_xy+max_xy)/2])
#             if pic_times == 10:
#                 SpecifyBall_mid = SpecifyBall_mid/10
#                 CueBalll_mid = CueBalll_mid/10
#                 ##image to real
#                 # SpecifyBall_mid = SpecifyBall_mid
#                 # CueBalll_mid = CueBalll_mid
#                 GetInfoFlag = True
#     pic_times+=1
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

    #Obj_Data_Calculation()

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
    for case in switch(MissionType_Flag): #傳送指令給手臂動作
        if case(0):
            Type = MissionType.Pick
            MissionType_Flag +=1
            break
        if case(1):
            Type = MissionType.Place
            MissionType_Flag -=1
            break
    CurrentMissionType = Type
    return Type

def MissionItem(ItemNo):
    global MotionKey
    Key_PickCommand = [\
        Arm_cmd.MoveToObj_Pick,\
        Arm_cmd.Absort_ON,\
        Arm_Stop,\
        ]
    Key_PlaceCommand = [\
        Arm_cmd.MoveToTarget_Place,\
        Arm_cmd.Absort_OFF,\
        Arm_Stop,\
        ]
    for case in switch(ItemNo): 
        if case(MissionType.Pick):
            MotionKey = Key_PickCommand
            break
        if case(MissionType.Place):
            MotionKey = Key_PlaceCommand
            break
    return MotionKey

def Execute_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionStep,MotionSerialKey,MissionEndFlag,CurrentMissionType
    Arm_state = robot_ctr.get_robot_motion_state() ## get arm state
    if Arm_state == 1:  
        if MotionKey[MotionStep] == Arm_cmd.Arm_Stop:
            if MissionEndFlag == True:
                CurrentMissionType = MissionType.Mission_End
                GetInfoFlag = False
                GetKeyFlag = False
                ExecuteFlag = False
                print("Mission_End")
            elif CurrentMissionType == MissionType.Pick:
                GetInfoFlag = False
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
                print("Pick")
            elif CurrentMissionType == MissionType.Get_Img:
                GetInfoFlag = True
                GetKeyFlag = False
                ExecuteFlag = False
                MotionStep = 0
        else:
            MotionItem(MotionSerialKey[MotionStep])
            MotionStep += 1
## pick down to box base [0 ,45.4, -40.7, -180,0,90]
## pick up to box above [0 ,45.4, -6.4, -180,0,90]
## place up target [39.4 ,1.09, -6.4, -180,0,90]
## place down target [39.4 ,1.09, -30.6, -180,0,90]
def MotionItem(ItemNo):
    global SpeedValue,PushFlag,MissionEndFlag,CurrentMissionType
    SpeedValue = 5 # test speed 
    for case in switch(ItemNo): #傳送指令給socket選擇手臂動作
        if case(Arm_cmd.Arm_Stop):
            
            print("Arm_Stop")
            break
        if case(Arm_cmd.MoveToObj_Pick):
            positon = [0 ,45.4, -6.4, -180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            positon = [0 ,45.4, -40.7, -180,0,90]
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test
            
            print("MoveToObj_Pick")
            break
        if case(Arm_cmd.Absort_ON):
            robot_ctr.Set_digital_output(1,True)
            print("Absort_ON")
            break
        if case(Arm_cmd.MoveToObj_PickUp):
            positon = [0 ,45.4, -6.4, -180,0,90]
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test
            print("MoveToObj_Pick")
            break
        if case(Arm_cmd.MoveToTarget_Place):
            #SpeedValue = 10
            positon = [39.4 ,1.09, -6.4, -180,0,90]
            robot_ctr.Step_AbsPTPCmd(positon)
            positon = [39.4 ,1.09, -30.6, -180,0,90]
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test

            print("MoveToTarget_Place")
            break
        if case(Arm_cmd.Absort_OFF):
            robot_ctr.Set_digital_output(1,False)
            print("Absort_OFF")
            break
        if case(Arm_cmd.MoveToTarget_PlaceUp):
            #SpeedValue = 10
            positon = [39.4 ,1.09, -6.4, -180,0,90]
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test

            print("MoveToTarget_Place")
            break
        if case(Arm_cmd.Get_obj):
            CurrentMissionType = MissionType.Get_Img
            ##任務結束旗標
            MissionEndFlag = True
            robot_ctr.Go_home()
            print("MissionEnd")
            break
        if case(): 
            print ("something else!")
##-------------strategy end ------------


## test use
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

