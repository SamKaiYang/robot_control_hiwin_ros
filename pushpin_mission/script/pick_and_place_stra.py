#### ros cmd
#roslaunch pushpin_mission pick_and_place.launch
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
from pushpin_mission.msg import ROI_array
#ROS message sent format
from std_msgs.msg import String
#Hiwin arm api class
from control_node import HiwinRobotInterface
#pixel_z to base
from hand_eye.srv import eye2base, eye2baseRequest
#avoidance
from collision_avoidance.srv import collision_avoid, collision_avoidRequest

DEBUG = True  # Set True to show debug log, False to hide it.
ItemNo = 0
positon = [0.0,0.0,10.0,-180,0,0]
Goal = [0.0,0.0,10.0,-180,0,0]

##-----Mission dataset
GetInfoFlag = False
ExecuteFlag = False
GetKeyFlag = False
MissionEndFlag = False
MotionSerialKey = []
MissionType_Flag = 0
MotionStep = 0
CurrentMissionType = 0
arm_move_times = 1

###---pixel z to base data init
camera_z = 42

## strategy data init 
pick_obj_times = 0
next_take_yolo_flag = False

class Arm_cmd(enum.IntEnum):
    MoveToObj_Pick = 1
    MoveToTarget_Place = 2
    Absort_ON = 3
    Absort_OFF = 4
    MoveToObj_PickUp = 5
    MoveToTarget_PlaceUp = 6
    Absort_Check = 7
    Arm_Stop = 8
    Get_Image1 = 9
    Get_Image2 = 10
    Go_back_home = 11

class MissionType(enum.IntEnum):
    Get_Img = 0
    Pick = 1
    Place = 2
    # new second take pic point
    Get_Img2 = 3
    Mission_End = 4
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
    def __init__(self,probability,x,y,id_name,Class_name):
        self.probability = probability
        self.x = x
        self.y = y
        self.id_name = str(id_name)
        self.Class_name = str(Class_name)
        self.data = []
    def add(self):
        self.data.append([x,y])
    def remove_data(self):
        self.data = []

boxes = bounding_boxes(0,0,0,0,0)
# YOLO V4 input
def Yolo_callback(data):
    global obj_num,pick_obj_times
    check_obj_count = 0
    obj_num = len((data.ROI_list))
    if obj_num == 0:
        print("No Object Found!")
        print("change method to Realsense!")
    elif obj_num != 0 and next_take_yolo_flag == True:
        next_take_yolo_flag = False
        for i in range(obj_num):
            boxes.probability = data.ROI_list[i].probability
            if boxes.probability >=0.9:
                boxes.x = data.ROI_list[i].x
                boxes.y = data.ROI_list[i].y
                boxes.id_name = data.ROI_list[i].id
                boxes.Class_name = data.ROI_list[i].object_name

                boxes.x = data.ROI_list[i].x
                boxes.y = data.ROI_list[i].y
                boxes.id_name = data.ROI_list[i].id
                boxes.Class_name = data.ROI_list[i].object_name
                boxes.add(boxes.x,boxes.y)
                check_obj_count += 1
            pick_obj_times = check_obj_count
                # print("boxes.x:",boxes.x)
                # print("boxes.y",boxes.y)
                # print("boxes.id_name",boxes.id_name)
                # print("boxes.Class_name",boxes.Class_name)
                ########test #####
                #Obj_Data_Calculation()
def Obj_Data_Calculation(obj_times):
    baseRequest = eye2baseRequest()
    baseRequest.ini_pose = [boxes.data[obj_times][0],boxes[obj_times][1],camera_z] 
    target_base = pixel_z_to_base_client(baseRequest) #[x,y,z]
    avoidRequest = collision_avoidRequest()
    avoidRequest.ini_pose = [target_base[0],target_base[1],target_base[2],180,0,0] 
    avoidRequest.limit = 0.1 # test
    avoidRequest.dis = 10 # test 
    target_base_avoidance = base_avoidance_client(avoidRequest)
    print("target_base:",target_base)
    print("target_base_avoidance:",target_base_avoidance)

#### origin 0916 v1#### origin 0916 v1#### origin 0916 v1#### origin 0916 v1#### origin 0916 v1
# def Yolo_callback(data):
#     global obj_num,pick_obj_times
#     check_obj_count = 0
#     obj_num = len((data.ROI_list))
#     if obj_num == 0:
#         print("No Object Found!")
#         print("change method to Realsense!")
#     else:
#         for i in range(obj_num):
#             boxes.probability = data.ROI_list[i].probability
#             if boxes.probability >=0.9:
#                 boxes.x = data.ROI_list[i].x
#                 boxes.y = data.ROI_list[i].y
#                 boxes.id_name = data.ROI_list[i].id
#                 boxes.Class_name = data.ROI_list[i].object_name

#                 boxes.x = data.ROI_list[i].x
#                 boxes.y = data.ROI_list[i].y
#                 boxes.id_name = data.ROI_list[i].id
#                 boxes.Class_name = data.ROI_list[i].object_name
#                 boxes.add()
#                 check_obj_count += 1
#             pick_obj_times = check_obj_count
#                 # print("boxes.x:",boxes.x)
#                 # print("boxes.y",boxes.y)
#                 # print("boxes.id_name",boxes.id_name)
#                 # print("boxes.Class_name",boxes.Class_name)
#                 ########test #####
#                 #Obj_Data_Calculation()
##-------------strategy start ------------
# def Obj_Data_Calculation():
#     baseRequest = eye2baseRequest()
#     baseRequest.ini_pose = [boxes.x,boxes.y,camera_z] 
#     target_base = pixel_z_to_base_client(baseRequest) #[x,y,z]
#     avoidRequest = collision_avoidRequest()
#     avoidRequest.ini_pose = [target_base[0],target_base[1],target_base[2],180,0,0] 
#     avoidRequest.limit = 0.1 # test
#     avoidRequest.dis = 10 # test 
#     target_base_avoidance = base_avoidance_client(avoidRequest)
#     print("target_base:",target_base)
#     print("target_base_avoidance:",target_base_avoidance)
#### origin 0916 v1#### origin 0916 v1#### origin 0916 v1#### origin 0916 v1
def pixel_z_to_base_client(pixel_to_base):
    rospy.wait_for_service('robot/pix2base')
    try:
        pixel_z_to_base = rospy.ServiceProxy('robot/pix2base', eye2base)
        resp1 = pixel_z_to_base(pixel_to_base)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def base_avoidance_client(target_base_to_avoidance):
    rospy.wait_for_service('robot/easy_CA')
    try:
        base_to_avoidance = rospy.ServiceProxy('robot/easy_CA', collision_avoid)
        resp1 = base_to_avoidance(target_base_to_avoidance)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def Mission_Trigger():
    if GetInfoFlag == True and GetKeyFlag == False and ExecuteFlag == False:
        GetInfo_Mission()
    if GetInfoFlag == False and GetKeyFlag == True and ExecuteFlag == False:
        GetKey_Mission()
    if GetInfoFlag == False and GetKeyFlag == False and ExecuteFlag == True:
        Execute_Mission()
def GetInfo_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag

    Obj_Data_Calculation()

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
    for case in switch(MissionType_Flag):
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
        Arm_cmd.Absort_Check,\
        Arm_cmd.MoveToObj_PickUp,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_PlaceCommand = [\
        Arm_cmd.MoveToTarget_Place,\
        Arm_cmd.Absort_OFF,\
        Arm_cmd.MoveToTarget_PlaceUp,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_Get_Image1_Command = [\
        Arm_cmd.Get_Image1,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_Get_Image2_PlaceCommand = [\
        Arm_cmd.Get_Image2,\
        Arm_cmd.Arm_Stop,\
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
            elif CurrentMissionType == MissionType.Place:
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
            #MotionStep += 1
def MotionItem(ItemNo):
    global SpeedValue,PushFlag,MissionEndFlag,CurrentMissionType,MotionStep
    robot_ctr.Set_override_ratio(5) # test speed 
    for case in switch(ItemNo):
        if case(Arm_cmd.Arm_Stop):
            
            print("Arm_Stop")
            break
        if case(Arm_cmd.MoveToObj_Pick):
            ###above box height
            above_box_height = 10
            positon = [target_base_avoidance[0],target_base_avoidance[1],above_box_height,target_base_avoidance[3],target_base_avoidance[4],target_base_avoidance[5]] ###target obj position above
            robot_ctr.Step_AbsPTPCmd(positon)
            positon = [target_base_avoidance[0],target_base_avoidance[1],target_base_avoidance[2]+10,target_base_avoidance[3],target_base_avoidance[4],target_base_avoidance[5]] ###target obj position
            robot_ctr.Step_AbsPTPCmd(positon)
            positon = target_base_avoidance
            #robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test
            print("MoveToObj_Pick")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_ON):
            robot_ctr.Set_digital_output(1,True)
            print("Absort_ON")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_Check):
            robot_inputs_state = robot_ctr.Get_current_robot_inputs()
            if robot_inputs_state[0] == True:
                print("Absort success") 
                MotionStep += 1
            else:
                print("Absort fail")
                #MotionStep += 1 # tmp
            break
        if case(Arm_cmd.MoveToObj_PickUp):
            positon = [target_base_avoidance[0],target_base_avoidance[1],target_base_avoidance[2]+10,target_base_avoidance[3],target_base_avoidance[4],target_base_avoidance[5]] ###target obj position
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10)
            above_box_height = 10
            positon = [target_base_avoidance[0],target_base_avoidance[1],above_box_height,target_base_avoidance[3],target_base_avoidance[4],target_base_avoidance[5]] ###target obj position above
            robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test
            print("MoveToObj_Pick")
            MotionStep += 1
            break
        if case(Arm_cmd.MoveToTarget_Place):
            # relate 0 point x+8 y-3 above box z +10
            positon = [8 ,-3, 10, -180,0,0]
            robot_ctr.Step_AbsPTPCmd(positon)
            print("MoveToTarget_Place")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_OFF):
            robot_ctr.Set_digital_output(1,False)
            print("Absort_OFF")
            MotionStep += 1
            break
        if case(Arm_cmd.MoveToTarget_PlaceUp):
            # positon = [39.4 ,1.09, -6.4, -180,0,90]
            # robot_ctr.Step_AbsLine_PosCmd(positon,1,10) ## test
            # print("MoveToTarget_Place")
            MotionStep += 1
            break
        if case(Arm_cmd.Get_Image1):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic point(1)
            positon =  [16.8611, 22.7639, -1.5206, -179.725, 10.719, -89.858]
            robot_ctr.Step_AbsPTPCmd(positon)
            MotionStep += 1
            break
        if case(Arm_cmd.Get_Image2):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic point(2)
            positon =  [16.8611, 37, -1.5206, -179.725, 10.719, -89.858]
            robot_ctr.Step_AbsPTPCmd(positon)
            MotionStep += 1
            break
        if case(Arm_cmd.Go_back_home):
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Go_home()
            print("MissionEnd")
            MotionStep += 1
            break
        if case(): 
            print ("something else!")
##-------------strategy end ------------
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
    a = rospy.Subscriber("obj_position",ROI_array,Yolo_callback)

    ## strategy trigger
    GetInfoFlag = True #Test no data
    try:
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            # set tool & base coor
            # tool_coor = [0,0,180,0,0,0]
            # base_coor = [0,0,0,0,0,0]
            robot_ctr.Set_base_number(5)
            #base_result = robot_ctr.Define_base(1,base_coor)
            robot_ctr.Set_tool_number(15)
            #tool_result = robot_ctr.Define_tool(1,tool_coor)

            robot_ctr.Set_operation_mode(1)
            robot_ctr.Set_override_ratio(5)
            robot_ctr.Set_acc_dec_ratio(100)

            robot_ctr.Set_digital_output(1,False)
            robot_ctr.Set_digital_output(2,False)
            robot_ctr.Set_digital_output(3,False)

        while(1):
            ### mission test 0916
            Mission_Trigger()
            if CurrentMissionType == MissionType.Mission_End:
                rospy.on_shutdown(myhook)

        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass
