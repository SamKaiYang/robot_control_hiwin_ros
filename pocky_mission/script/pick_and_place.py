#### ros cmd
#roslaunch pocky_mission pick_and_place.launch
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
# from pocky_mission.msg import ROI_bottom
# from pocky_mission.msg import ROI_top
# from pocky_mission.msg import ROI_array_bottom
# from pocky_mission.msg import ROI_array_top
# from pocky_mission.msg import vision_state
#ROS message sent format
from std_msgs.msg import String
#Hiwin arm api class
from control_node import HiwinRobotInterface
#pixel_z to base
from hand_eye.srv import eye2base, eye2baseRequest
#pocky strategy data srv
from pocky_mission.srv import pocky_data, pocky_dataResponse
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
#To be tested
bottom_camera_z = 53
top_camera_z = 55
# camera_z = 53 

## strategy data init 
bottom_boxes_List = []
top_boxes_List = []
pick_obj_times = 0
target_base = []
arm_down_pick_flag = False
Stop_motion_flag = False
vision_state_flag = False
objects_picked_num = 0#Number of objects picked

LineDown_Speed = 3
ArmGernel_Speed = 5

class Arm_cmd(enum.IntEnum):
    MoveToObj_Pick1 = 1
    MoveToTarget_Place = 2
    Absort_ON = 3
    Absort_OFF = 4
    MoveToObj_PickUp = 5
    MoveToTarget_PlaceUp = 6
    Absort_Check = 7
    Arm_Stop = 8
    Get_Image = 9
    Go_Image1 = 10
    Go_back_home = 12
    MoveToObj_Pick2 = 13

class MissionType(enum.IntEnum):
    Get_Img = 0
    Pick = 1
    Place = 2
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

class top_boxes_class():
    def __init__(self,top_box,top_CenterX,top_CenterY,top_Angle):
        self.top_box = top_box
        self.top_CenterX = top_CenterX
        self.top_CenterY = top_CenterY
        self.top_Angle = top_Angle
        self.data = []
    def add(self):
        self.data.append([x,y])
    def remove_data(self):
        self.data = []

top_boxes = top_boxes_class(0,0,0,0)

class bottom_boxes_class():
    def __init__(self,bottom_box,bottom_CenterX,bottom_CenterY,bottom_Angle):
        self.bottom_box = bottom_box
        self.bottom_CenterX = bottom_CenterX
        self.bottom_CenterY = bottom_CenterY
        self.bottom_Angle = bottom_Angle
        self.data = []
    def add(self):
        self.data.append([x,y])
    def remove_data(self):
        self.data = []

bottom_boxes = bottom_boxes_class(0,0,0,0)

# def top_obj_data_callback(data):
#     global top_boxes_List,add_top_data_flag
#     obj_num = len((data.ROI_top_list))
#     if obj_num == 0:
#         print("change method to Realsense!")
#     else:
#         if add_top_data_flag == True:
#             for i in range(obj_num):
#                 top_boxes.top_box = data.ROI_top_list[i].top_box
#                 top_boxes.top_CenterX = data.ROI_top_list[i].top_CenterX
#                 top_boxes.top_CenterY = data.ROI_top_list[i].top_CenterY
#                 top_boxes.top_Angle = data.ROI_top_list[i].top_Angle
#                 top_boxes_List.append(top_boxes)
#             add_top_data_flag == False

# def bottom_obj_data_callback(data):
#     global bottom_boxes_List,add_bottom_data_flag
#     obj_num = len((data.ROI_bottom_list))
#     if obj_num == 0:
#         print("change method to Realsense!")
#     else:
#         if add_bottom_data_flag == True:
#             for i in range(obj_num):
#                 bottom_boxes.bottom_box = data.ROI_bottom_list[i].bottom_box
#                 bottom_boxes.bottom_CenterX = data.ROI_bottom_list[i].bottom_CenterX
#                 bottom_boxes.bottom_CenterY = data.ROI_bottom_list[i].bottom_CenterY
#                 bottom_boxes.bottom_Angle = data.ROI_bottom_list[i].bottom_Angle
#                 bottom_boxes_List.append(bottom_boxes)
#             add_bottom_data_flag == False
# def vision_state_callback(data):
#     global vision_state_flag 
#     vision_state_flag = data.state

def Obj_Data_Calculation(pixel_x,pixel_y,camera_z):  #Enter the number of objects that have been picked and place
    global objects_picked_num,target_base
    baseRequest = eye2baseRequest()
    baseRequest.ini_pose = [pixel_x,pixel_y,camera_z] ## test
    target_base = pixel_z_to_base_client(baseRequest) #[x,y,z]

def pixel_z_to_base_client(pixel_to_base):
    rospy.wait_for_service('robot/pix2base')
    try:
        pixel_z_to_base = rospy.ServiceProxy('robot/pix2base', eye2base)
        resp1 = pixel_z_to_base(pixel_to_base)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pocky_data_client(data):
    rospy.wait_for_service('pocky_service')
    try:
        pocky_data = rospy.ServiceProxy('pocky_service', pocky_data_sent)
        resp = pocky_data(data)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def Mission_Trigger():
    if GetKeyFlag == True and ExecuteFlag == False:
        GetKey_Mission()
    if GetKeyFlag == False and ExecuteFlag == True:
        Execute_Mission()

def GetKey_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionSerialKey

    Mission = Get_MissionType()
    MissionItem(Mission)
    MotionSerialKey = MotionKey
    GetKeyFlag = False
    ExecuteFlag = True

def Get_MissionType():
    global MissionType_Flag,CurrentMissionType
    for case in switch(MissionType_Flag):
        if case(MissionType.Pick):
            Type = MissionType.Pick
            MissionType_Flag = MissionType.Place
            break
        if case(MissionType.Place):
            Type = MissionType.Place
            MissionType_Flag = MissionType.Pick
            break
        if case(MissionType.Get_Img):
            Type = MissionType.Get_Img
            break
        if case(MissionType.Mission_End):
            Type = MissionType.Mission_End
            break
    CurrentMissionType = Type
    return Type

def MissionItem(ItemNo):
    global MotionKey
    Key_PickCommand = [\
        Arm_cmd.MoveToObj_Pick1,\
        Arm_cmd.MoveToObj_Pick2,\
        Arm_cmd.MoveToObj_PickUp,\
        Arm_cmd.Absort_Check,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_PlaceCommand = [\
        Arm_cmd.MoveToTarget_PlaceUp,\
        Arm_cmd.MoveToTarget_Place,\
        Arm_cmd.Absort_OFF,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_Get_Image1_Command = [\
        Arm_cmd.Go_Image1,\
        Arm_cmd.Get_Image,\
        Arm_cmd.Arm_Stop,\
        ]
    Key_Mission_End_Command = [\
        Arm_cmd.Go_back_home,\
        Arm_cmd.Arm_Stop,\
        ]
    for case in switch(ItemNo): 
        if case(MissionType.Pick):
            MotionKey = Key_PickCommand
            break
        if case(MissionType.Place):
            MotionKey = Key_PlaceCommand
            break
        if case(MissionType.Get_Img):
            MotionKey = Key_Get_Image1_Command
            break
        if case(MissionType.Mission_End):
            MotionKey = Key_Mission_End_Command
            break
    return MotionKey

def Execute_Mission():
    global GetInfoFlag,GetKeyFlag,ExecuteFlag,MotionKey,MotionStep,MotionSerialKey,MissionEndFlag,CurrentMissionType,arm_down_pick_flag
    global target_base_avoidance,Stop_motion_flag
    
    Arm_state = robot_ctr.get_robot_motion_state() ## get arm state
    if arm_down_pick_flag == True and Arm_state == 2:
        robot_inputs_state = robot_ctr.Get_current_robot_inputs() # Determine whether the object is sucked
        if robot_inputs_state[0] == True:  # is digital IO input 1 pin
            print("Absort success") 
            robot_ctr.Stop_motion()  #That is, it is sucked and started to place
            time.sleep(0.1)

            Stop_motion_flag = True
            arm_down_pick_flag = False
        else:
            pass  # Continue task
    Arm_state = robot_ctr.get_robot_motion_state() ## get arm state
    if Arm_state == 1:  
        if MotionKey[MotionStep] == Arm_cmd.Arm_Stop:
            if MissionEndFlag == True:
                CurrentMissionType = MissionType.Mission_End
                GetKeyFlag = False
                ExecuteFlag = False
                print("Mission_End")
            elif CurrentMissionType == MissionType.Pick:
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
                print("Pick")
            elif CurrentMissionType == MissionType.Place:
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
                print("Pick")
            elif CurrentMissionType == MissionType.Get_Img:
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
        else:
            MotionItem(MotionSerialKey[MotionStep])
def MotionItem(ItemNo):
    global SpeedValue,PushFlag,MissionEndFlag,CurrentMissionType,MotionStep,objects_picked_num,MissionType_Flag
    global target_base_avoidance,target_base_above_avoidance,arm_down_pick_flag,Stop_motion_flag,Absort_fail_to_Get_image
    global LineDown_Speed, ArmGernel_Speed
    global bottom_boxes_List,top_boxes_List
    for case in switch(ItemNo):
        if case(Arm_cmd.Arm_Stop):
            print("Arm_Stop")
            break
        ## To be tested
        if case(Arm_cmd.MoveToObj_Pick1):
            # if len(bottom_List) != 0:
            #     if bottom_List[0][0] == 'WP':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif bottom_List[0][0] == 'GP':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif bottom_List[0][0] == 'Y':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif bottom_List[0][0] == 'G':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif bottom_List[0][0] == 'W':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif bottom_List[0][0] == 'R':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            # if len(top_List) != 0:
            #     if top_List[0][0] == 'WP':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif top_List[0][0] == 'GP':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif top_List[0][0] == 'Y':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif top_List[0][0] == 'G':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif top_List[0][0] == 'W':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            #     elif top_List[0][0] == 'R':
            #         positon = [0,0,10,180,0,0]
            #         robot_ctr.Step_AbsPTPCmd(positon)
            print("MoveToObj_Pick1")
            MotionStep += 1
            break
        if case(Arm_cmd.MoveToObj_Pick2):
            positon = [0,0,-10,0,0,0] ###target obj position
            robot_ctr.Step_RelLineCmd(positon,0,10)
            robot_ctr.Set_override_ratio(LineDown_Speed) ##speed low

            arm_down_pick_flag = True
            robot_ctr.Set_digital_output(1,True) # Absort_ON
            print("MoveToObj_Pick2")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_Check):
            robot_inputs_state = robot_ctr.Get_current_robot_inputs() # Determine whether the object is sucked
            if robot_inputs_state[0] == True:  # is digital IO input 1 pin
                print("Absort success check and mission continue") 
                MotionStep += 1
            else:
                print("Absort fail and mission continue to Get image")
                robot_ctr.Set_digital_output(1,False) # Absort_OFF
                # MissionType_Flag =  MissionType.Get_Img
                # GetKeyFlag = True
                # ExecuteFlag = False
                MotionStep += 1 # tmp
            break
        if case(Arm_cmd.MoveToObj_PickUp):
            if Stop_motion_flag == True: #There are early pick up items
                print("Absort success fucccccckkkkk") 
            # else: # Did not pick up items early
            positon = [0,0,10,0,0,0] 
            robot_ctr.Step_RelLineCmd(positon,0,10)
            robot_ctr.Set_override_ratio(ArmGernel_Speed)
            arm_down_pick_flag = False #Initialize the flag to determine the next action 
            print("MoveToObj_PickUp")
            MotionStep += 1
            break
        ## To be tested
        if case(Arm_cmd.MoveToTarget_Place):
            positon = [0 ,0, -10, 0,0,0]
            robot_ctr.Step_RelLineCmd(positon)
            print("MoveToTarget_Place")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_OFF):
            robot_ctr.Set_digital_output(1,False)
            time.sleep(0.1)
            print("Absort_OFF")
            MotionStep += 1
            break
        ## To be tested
        if case(Arm_cmd.MoveToTarget_PlaceUp):
             
            # if len(bottom_List) != 0:
            #     Obj_Data_Calculation(bottom_List[0][1],bottom_List[0][2],bottom_camera_z)
            #     positon = [target_base[0],target_base[1],0,180,0,bottom_List[0][3]] ## Z test
            #     robot_ctr.Step_AbsPTPCmd(positon)
            #     #delete bottom 1 object
            #     del bottom_List[0]
            # if len(bottom_List) == 0 and len(top_List) != 0:
            #     Obj_Data_Calculation(top_List[0][1],top_List[0][2],top_camera_z)
            #     positon = [target_base[0],target_base[1],0,180,0,top_List[0][3]] ## Z test
            #     robot_ctr.Step_AbsPTPCmd(positon)
            #     #delete top 1 object
            #     del top_List[0]
            # elif len(bottom_List) == 0 and len(top_List) == 0:
            #     MissionType_Flag = MissionType.Mission_End
            #     # robot_ctr.Stop_motion()  #That is, it is sucked and started to place
            #     print("mission end")
            MotionStep += 1
            break
        if case(Arm_cmd.Go_Image1):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic point(1)
            positon =  [11.3440, 26.4321, 11.23, 179.994, 10.002, -0.488]
            robot_ctr.Step_AbsPTPCmd(positon)
            # time.sleep(20) ### test 9/16
            MotionStep += 1
            break
        if case(Arm_cmd.Get_Image):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic
            time.sleep(0.3) # Delayed time to see
            # add_bottom_data_flag = True
            # add_top_data_flag = True
            if vision_state_flag == True:
                # bottom_List = bottom_boxes_List
                # top_List = top_boxes_List
                MissionType_Flag = MissionType.Pick
                MotionStep += 1
                # vision_state_flag = False
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
    a = rospy.Subscriber("top_obj_data",ROI_array_bottom,bottom_obj_data_callback)
    b = rospy.Subscriber("bottom_obj_data",ROI_array_top,top_obj_data_callback)
    vision = rospy.Subscriber("vision_state",vision_state,vision_state_callback)

    ## strategy trigger
    try:
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Set_base_number(5)
            robot_ctr.Set_tool_number(15) #transparent tool

            robot_ctr.Set_operation_mode(1)
            
            ArmGernel_Speed = 100
            LineDown_Speed = 10
            robot_ctr.Set_override_ratio(ArmGernel_Speed)

            robot_ctr.Set_acc_dec_ratio(100)
            robot_ctr.Set_digital_output(1,False)
            robot_ctr.Set_digital_output(2,False)
            robot_ctr.Set_digital_output(3,False)

            GetKeyFlag = True # start strategy
            # Get_Image = 0 ,so first take a photo to see if there are objects
        start_input = int(input('For first strategy, press 1 For pocky service test, press 2 \n'))

        if start_input == 1:
            while(1):
                Mission_Trigger()
                if CurrentMissionType == MissionType.Mission_End:
                    rospy.on_shutdown(myhook)
        if start_input == 2:
            while(1):
                aa = pocky_data_client(1)
                print(aa)

        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass
