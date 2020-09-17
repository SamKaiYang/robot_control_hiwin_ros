import rospy
import enum
import time
import numpy as np
from control_node import HiwinRobotInterface
from collision_avoidance.srv import collision_avoid, collision_avoidRequest
from tool_angle.srv import ax_cali, ax_caliRequest, tool_angle, tool_angleRequest

pic_pos = [
[5,  0.5,  -23,  90, 0, 0],
[10, 0.5,  -23,  90, 0, 0],
[15, 0.5,  -23,  90, 0, 0],
[20, 0.5,  -23,  90, 0, 0],
[25, 0.5,  -23,  90, 0, 0],
[30, 0.5,  -23,  90, 0, 0],
[39, 0.5,  -23,  90, 0, 0],
[39, 10,   -23,  180, -90, 0],
[39, 15,   -23,  180, -90, 0],
[39, 20,   -23,  180, -90, 0],
[39, 25,   -23,  180, -90, 0],
[39, 30,   -23,  180, -90, 0],
[39, 35,   -23,  180, -90, 0],
[39, 40,   -23,  180, -90, 0],
[39, 45,   -23,  180, -90, 0],
[39, 50,   -23,  180, -90, 0],
[39, 58.2, -23, -90, 0, 0],
[30, 58.2, -23, -90, 0, 0],
[25, 58.2, -23, -90, 0, 0],
[20, 58.2, -23, -90, 0, 0],
[15, 58.2, -23, -90, 0, 0],
[10, 58.2, -23, -90, 0, 0],
[1.2, 58.2,-23, -90, 0, 0],
[1.2, 50,  -23,  180, 90, 0],
[1.2, 45,  -23,  180, 90, 0],
[1.2, 40,  -23,  180, 90, 0],
[1.2, 35,  -23,  180, 90, 0],
[1.2, 30,  -23,  180, 90, 0],
[1.2, 25,  -23,  180, 90, 0],
[1.2, 20,  -23,  180, 90, 0],
[1.2, 15,  -23,  180, 90, 0],
[1.2, 10,  -23,  180, 90, 0],
[1.2, 5,   -23,  180, 90, 0]]

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

    def CA_client(self, req):
        rospy.wait_for_service('/robot/_CA')
        try:
            CA = rospy.ServiceProxy('/robot/_CA', collision_avoid)
            res = CA(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def tool_angle_client(self, req):
        rospy.wait_for_service('/tool/tool_angle')
        try:
            client = rospy.ServiceProxy('/tool/tool_angle', tool_angle)
            res = client(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def tool_cali_client(self, req):
        rospy.wait_for_service('/tool/ax_cali')
        try:
            client = rospy.ServiceProxy('/tool/ax_cali', ax_cali)
            res = client(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def Mission_Trigger(self):
        if self.arm_move == True and robot_ctr.get_robot_motion_state() == Arm_status.Isbusy:
            self.arm_move = False
        # if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        if robot_ctr.get_robot_motion_state() == Arm_status.Idle and self.arm_move == False:
            if self.state == State.move:
                pos = self.pos[0]
                # position = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]]
                req = collision_avoidRequest()
                req.ini_pose = pos
                req.limit = 5
                req.dis = 3
                res = self.CA_client(req)
                position = np.array(res.tar_pose)
                req = tool_angleRequest()
                req.angle = str(res.suc_angle)
                self.tool_angle_client(res.suc_angle)
                robot_ctr.Set_ptp_speed(30)
                robot_ctr.Step_AbsPTPCmd(position)
                self.pos = np.delete(self.pos, 0, 0)
                self.state = State.take_pic
                self.arm_move = True

            elif self.state == State.take_pic:
                robot_ctr.Set_ptp_speed(30)
                robot_ctr.Step_AbsPTPCmd([16.5, 29.2, 3, -180, 0, 0])
                if len(self.pos) > 0:
                    self.state = State.move
                else:
                    self.state = State.finish
                    req = ax_caliRequest()
                    req.cmd = 'dis'
                    self.tool_cali_client(req)
                    return
                req = tool_angleRequest()
                req.angle = 0
                self.tool_angle_client(req)

if __name__ == "__main__":
    rospy.init_node('easy_CA_test')
    robot_ctr = HiwinRobotInterface(robot_ip="192.168.0.1", connection_level=1,name="manipulator")
    robot_ctr.connect()
    time.sleep(1)

    robot_ctr.Set_operation_mode(0)
    # set tool & base coor
    tool_coor = [0,0,0,0,0,0]
    base_coor = [0,0,0,0,0,0]
    robot_ctr.Set_base_number(5)
    # base_result = robot_ctr.Define_base(1,base_coor)
    robot_ctr.Set_tool_number(15)
    # tool_result = robot_ctr.Define_tool(1,tool_coor)
    robot_ctr.Set_operation_mode(1)
    robot_ctr.Set_override_ratio(30)
    poses = []
    strtage = EasyCATest()
    while strtage.state != State.finish and not rospy.is_shutdown():
        strtage.Mission_Trigger()
        time.sleep(0.1)

