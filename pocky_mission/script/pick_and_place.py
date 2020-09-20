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
#ROS message sent format
from std_msgs.msg import String
#Hiwin arm api class
from control_node import HiwinRobotInterface
#pixel_z to base
from hand_eye.srv import eye2base, eye2baseRequest
#pocky strategy data srv
from pocky_mission.srv import pocky_data, pocky_dataRequest
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
#bottom_camera_z = 53
single_camera_z = 56.5
double_camera_z = 54
bottom_count = 0
top_count = 0
## strategy data init 

pick_obj_times = 0
target_base = []
target_down_base = []
arm_down_pick_flag = False
Stop_motion_flag = False
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
    MoveToTarget_above = 14

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

def Obj_Data_Calculation(pixel_x,pixel_y,camera_z):  #Enter the number of objects that have been picked and place
    global target_base
    # print("pixel_x:",pixel_x)
    # print("pixel_y:",pixel_y)
    # print("camera_z:",camera_z)
    baseRequest = eye2baseRequest()
    baseRequest.ini_pose = [pixel_x,pixel_y,camera_z] ## test
    target_base = pixel_z_to_base_client(baseRequest) #[x,y,z]
    print("target_base:",target_base)

def Obj_Data_Calculation_down(pixel_x,pixel_y):  #Enter the number of objects that have been picked and place
    global target_down_base
    baseRequest = eye2baseRequest()
    baseRequest.ini_pose = [pixel_x,pixel_y] ## test
    target_down_base = down_camera_to_base_client(baseRequest) #[x,y]
    print("target_down_base:",target_down_base)
def pixel_z_to_base_client(pixel_to_base):
    rospy.wait_for_service('robot/fix_cam2base')
    try:
        pixel_z_to_base = rospy.ServiceProxy('robot/fix_cam2base', eye2base)
        resp1 = pixel_z_to_base(pixel_to_base)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
def down_camera_to_base_client(pixel_to_base):
    rospy.wait_for_service('robot/down_cam2base')
    try:
        pixel_z_to_base = rospy.ServiceProxy('robot/down_cam2base', eye2base)
        resp1 = pixel_z_to_base(pixel_to_base)
        return resp1.tar_pose
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def pocky_data_client(data):
    rospy.wait_for_service('/pocky_service')
    try:
        pocky_ = rospy.ServiceProxy('/pocky_service', pocky_data)
        resp = pocky_(data)
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
            print("place mission")
            break
        if case(MissionType.Place):
            Type = MissionType.Place
            MissionType_Flag = MissionType.Pick
            print("pick mission")
            break
        if case(MissionType.Get_Img):
            Type = MissionType.Get_Img
            print("Get image mission")
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
        Arm_cmd.MoveToTarget_above,\
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
    global target_base,target_down_base,Stop_motion_flag
    
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
            elif CurrentMissionType == MissionType.Place:
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
            elif CurrentMissionType == MissionType.Get_Img:
                GetKeyFlag = True
                ExecuteFlag = False
                MotionStep = 0
        else:
            MotionItem(MotionSerialKey[MotionStep])
def MotionItem(ItemNo):
    global SpeedValue,PushFlag,MissionEndFlag,CurrentMissionType,MotionStep,objects_picked_num,MissionType_Flag
    global target_down_base,target_base,arm_down_pick_flag,Stop_motion_flag,Absort_fail_to_Get_image
    global LineDown_Speed, ArmGernel_Speed
    global bottom_count,top_count
    for case in switch(ItemNo):
        if case(Arm_cmd.Arm_Stop):
            print("Arm_Stop")
            break
        ## To be tested
        if case(Arm_cmd.MoveToObj_Pick1):
            pocky = pocky_data_client(1)
            print(pocky)
            print("bottom_count:",bottom_count)
            print("top_count:",top_count)
            if bottom_count < len(pocky.bottom_box):
                if pocky.bottom_box[bottom_count] == 'WP':
                    positon = [2.9792,-28.4870,(-29.8462+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.bottom_box[bottom_count] == 'GP':
                    positon = [22.0686,-22.9382,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.bottom_box[bottom_count] == 'Y':
                    positon = [22.0686,-4.3049,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.bottom_box[bottom_count] == 'G':
                    positon = [22.0686,-13.1650,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.bottom_box[bottom_count] == 'W':
                    positon = [2.9792,-18.8779,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.bottom_box[bottom_count] == 'R':
                    positon = [2.9792,-8.8964,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                print("MoveToObj_bottom_box_Pick1")
            elif bottom_count == len(pocky.bottom_box) and top_count < len(pocky.top_box):
                if pocky.top_box[top_count] == 'WP':
                    positon = [2.9792,-28.4870,(-29.8462+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.top_box[top_count] == 'GP':
                    positon = [22.0686,-22.9382,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.top_box[top_count] == 'Y':
                    positon = [22.0686,-4.3049,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.top_box[top_count] == 'G':
                    positon = [22.0686,-13.1650,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.top_box[top_count] == 'W':
                    positon = [2.9792,-18.8779,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                elif pocky.top_box[top_count] == 'R':
                    positon = [2.9792,-8.8964,(-29.8414+10),180,0,0]
                    robot_ctr.Step_AbsPTPCmd(positon)
                    robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                print("MoveToObj_top_box_Pick1")
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
                MotionStep += 1 # tmp
            break
        if case(Arm_cmd.MoveToObj_PickUp):
            positon = [0,0,15,0,0,0] 
            robot_ctr.Step_RelLineCmd(positon,0,10)
            robot_ctr.Set_override_ratio(LineDown_Speed)
            arm_down_pick_flag = False #Initialize the flag to determine the next action 
            print("MoveToObj_PickUp")
            MotionStep += 1
            break
        ## To be tested
        if case(Arm_cmd.MoveToTarget_Place):
            positon = [0 ,0, -10, 0,0,0]
            robot_ctr.Step_RelLineCmd(positon)
            robot_ctr.Set_override_ratio(LineDown_Speed) ##speed low
            print("MoveToTarget_Place")
            MotionStep += 1
            break
        if case(Arm_cmd.Absort_OFF):
            robot_ctr.Set_digital_output(1,False)
            time.sleep(0.1)
            print("Absort_OFF")
            pocky = pocky_data_client(1)
            if bottom_count == len(pocky.bottom_box) and top_count == len(pocky.top_box):
                MissionType_Flag = MissionType.Mission_End
                bottom_count = 0
                top_count = 0
                print("mission end")
            MotionStep += 1
            break
        ## To be tested
        if case(Arm_cmd.MoveToTarget_PlaceUp):
            pocky = pocky_data_client(1)

            if bottom_count < len(pocky.bottom_box):
                Obj_Data_Calculation_down(pocky.bottom_CenterX[bottom_count],pocky.bottom_CenterY[bottom_count])
                positon = [target_down_base[0]+9,target_down_base[1]-32,-19.6414,180,0,pocky.bottom_Angle[bottom_count]] ## Z test
                robot_ctr.Step_AbsPTPCmd(positon)
                robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                bottom_count += 1
                print("MoveToTarget_bottom_box_PlaceUp")
            elif bottom_count == len(pocky.bottom_box) and top_count < len(pocky.top_box):
                # strategy double
                if bottom_count > 0:
                    Obj_Data_Calculation(pocky.top_CenterX[top_count],pocky.top_CenterY[top_count],double_camera_z)
                    positon = [target_base[0]+9,target_base[1]-32,-17,180,0,pocky.top_Angle[top_count]] ## Z test 
                # strategy single
                else:
                    Obj_Data_Calculation(pocky.top_CenterX[top_count],pocky.top_CenterY[top_count],single_camera_z)
                    positon = [target_base[0]+9,target_base[1]-32,-19.6414,180,0,pocky.top_Angle[top_count]] ## Z test 
                robot_ctr.Step_AbsPTPCmd(positon)
                robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
                top_count +=1
                print("MoveToTarget_top_box_PlaceUp")
            MotionStep += 1
            break
        ## To be tested
        if case(Arm_cmd.MoveToTarget_above):
            positon = [0 ,0, 15, 0,0,0]
            robot_ctr.Step_RelLineCmd(positon)
            robot_ctr.Set_override_ratio(LineDown_Speed) ##speed low
            print("MoveToTarget_above")
            MotionStep += 1
            break
        if case(Arm_cmd.Go_Image1):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic point(1)
            positon =  [11.3440, 36.4321, 11.23, 179.994, 10.002, -0.488]
            robot_ctr.Step_AbsPTPCmd(positon)
            robot_ctr.Set_override_ratio(ArmGernel_Speed) ##speed add
            # time.sleep(20) ### test 9/16
            MotionStep += 1
            print("Go_Image1")
            break
        if case(Arm_cmd.Get_Image):
            CurrentMissionType = MissionType.Get_Img
            ### test take pic
            time.sleep(0.3) # Delayed time to see
            pocky = pocky_data_client(1)
            if pocky.is_done == True:
                MissionType_Flag = MissionType.Pick
                MotionStep += 1
            print("Get_Image")
            break
        if case(Arm_cmd.Go_back_home):
            positon =  [11.3440, 36.4321, 11.23, 179.994, 10.002, -0.488]
            robot_ctr.Step_AbsPTPCmd(positon)
            robot_ctr.Set_override_ratio(10) ##speed add
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

    ## strategy trigger
    try:
        if robot_ctr.is_connected():
            robot_ctr.Set_operation_mode(0)
            robot_ctr.Set_base_number(5)
            robot_ctr.Set_tool_number(15) #transparent tool

            robot_ctr.Set_operation_mode(1)
            
            ArmGernel_Speed = 10
            LineDown_Speed = 5
            robot_ctr.Set_override_ratio(ArmGernel_Speed)

            robot_ctr.Set_acc_dec_ratio(100)
            robot_ctr.Set_digital_output(1,False)
            robot_ctr.Set_digital_output(2,False)
            robot_ctr.Set_digital_output(3,False)

            GetKeyFlag = True # start strategy
            # Get_Image = 0 ,so first take a photo to see if there are objects
        start_input = int(input('For first strategy, press 1 \nFor pocky service test, press 2 \nGo camera position, press 3\n'))

        if start_input == 1:
            while(1):
                Mission_Trigger()
                if CurrentMissionType == MissionType.Mission_End:
                    rospy.on_shutdown(myhook)
        if start_input == 2:
            pocky = pocky_data_client(1)
            List = []
            LLL = list(pocky.top_CenterX)
            while(1):
                # List = list(pocky)
                #LLL = list(pocky.top_CenterX)
                print(LLL)
                # del pocky.top_box[0]
                del LLL[0]
                print (LLL)
                # del pocky.top_CenterY[0]
                # del pocky.top_Angle[0]
                print('len:',len(pocky.top_box) )
                print(pocky.top_box)
                # print('fuck')
                # q = pocky_dataRequest()
                # q.request_flag = 1
                # print('fuckUU')
                # print(pocky)
        if start_input == 3:
            positon =  [11.3440, 36.4321, 11.23, 179.994, 10.002, -0.488]
            robot_ctr.Step_AbsPTPCmd(positon)
            print("Go_Image")
        rospy.spin()
    except KeyboardInterrupt:
        robot_ctr.Set_motor_state(0)
        robot_ctr.close()
        pass
