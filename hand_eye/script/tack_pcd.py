import rospy
import enum
import time
import numpy as np
from control_node import HiwinRobotInterface
from collision_avoidance.srv import collision_avoid, collision_avoidRequest
from hand_eye.srv import eye2base, eye2baseRequest
from hand_eye.srv import save_pcd, save_pcdRequest

pic_pos = [
[5,  0.5,  -20, -180, 0, 0],
[10, 0.5,  -20, -180, 0, 0],
[15, 0.5,  -20, -180, 0, 0],
[20, 0.5,  -20, -180, 0, 0],
[25, 0.5,  -20, -180, 0, 0],
[30, 0.5,  -20, -180, 0, 0],
[39, 0.5,  -20, -180, 0, 0],
[39, 10, -20, -180, 0, 0],
[39, 15, -20, -180, 0, 0],
[39, 20, -20, -180, 0, 0],
[39, 25, -20, -180, 0, 0],
[39, 30, -20, -180, 0, 0],
[39, 35, -20, -180, 0, 0],
[39, 40, -20, -180, 0, 0],
[39, 45, -20, -180, 0, 0],
[39, 50, -20, -180, 0, 0],
[39, 58.2, -20, -180, 0, 0],
[30, 58.2, -20, -180, 0, 0],
[25, 58.2, -20, -180, 0, 0],
[20, 58.2, -20, -180, 0, 0],
[15, 58.2, -20, -180, 0, 0],
[10, 58.2, -20, -180, 0, 0],
[1.2, 58.2,  -20, -180, 0, 0],
[1.2, 50,  -20, -180, 0, 0],
[1.2, 45,  -20, -180, 0, 0],
[1.2, 40,  -20, -180, 0, 0],
[1.2, 35,  -20, -180, 0, 0],
[1.2, 30,  -20, -180, 0, 0],
[1.2, 25,  -20, -180, 0, 0],
[1.2, 20,  -20, -180, 0, 0],
[1.2, 15,  -20, -180, 0, 0],
[1.2, 10,  -20, -180, 0, 0],
[1.2, 5,   -20, -180, 0, 0]]

class Arm_status(enum.IntEnum):
    Idle = 1
    Isbusy = 2

class State(enum.IntEnum):
    move = 0
    take_pic = 1
    finish = 2

class EasyCATest:
    def __init__(self):
        self.arm_move = False
        self.state = State.move
        self.pos = np.array(pic_pos)

    def hand_eye_client(self, req):
        rospy.wait_for_service('/robot/eye_trans2base')
        try:
            ez_ca = rospy.ServiceProxy('/robot/eye_trans2base', collision_avoid)
            res = ez_ca(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_pcd_client(self, req):
        rospy.wait_for_service('/get_pcd')
        try:
            ez_ca = rospy.ServiceProxy('/get_pcd', collision_avoid)
            res = ez_ca(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def Mission_Trigger(self):
        if self.arm_move == True and robot_ctr.get_robot_motion_state() == Arm_status.Isbusy:
            self.arm_move = False
        # if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        if robot_ctr.get_robot_motion_state() == Arm_status.Idle and self.arm_move == False:
            if self.state == State.move:
                print('ffffffffffffffffffffffffffffffffffffffffffffffff')
                pos = self.pos[0]
                # position = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]]
                
                print(pos)
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsLine_PosCmd(pos)
                self.pos = np.delete(self.pos, 0, 0)
                self.state = State.take_pic
                self.arm_move = True

            elif self.state == State.take_pic:
                req = eye2baseRequest()
                req.ini_pose = [0,0,0,0,0,0,]
                trans = self.hand_eye_client(req).tar_pose
                req = save_pcdRequest()
                req.curr_trans = trans
                req.name = 'mdfk'                               
                if len(self.pos) > 0:
                    self.state = State.move
                    req.save_mix = False
                else:
                    self.state = State.finish
                    req.save_mix = True
                self.get_pcd_client(req)

if __name__ == "__main__":
    rospy.init_node('get_pcd')
    robot_ctr = HiwinRobotInterface(robot_ip="192.168.0.1", connection_level=1,name="manipulator")
    robot_ctr.connect()


    robot_ctr.Set_operation_mode(0)
    # set tool & base coor
    tool_coor = [0,0,0,0,0,0]
    base_coor = [0,0,0,0,0,0]
    robot_ctr.Set_base_number(5)
    # base_result = robot_ctr.Define_base(1,base_coor)
    robot_ctr.Set_tool_number(15)
    # tool_result = robot_ctr.Define_tool(1,tool_coor)
    robot_ctr.Set_operation_mode(1)
    robot_ctr.Set_override_ratio(20)
    poses = []
    strtage = EasyCATest()
    while strtage.state != State.finish and not rospy.is_shutdown():
        strtage.Mission_Trigger()
        time.sleep(0.1)

