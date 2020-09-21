import rospy
import enum
import time
import numpy as np
import tf
from math import degrees, radians, cos
from control_node import HiwinRobotInterface
from collision_avoidance.srv import collision_avoid, collision_avoidRequest
from hand_eye.srv import eye2base, eye2baseRequest
from hand_eye.srv import save_pcd, save_pcdRequest
from avoidance_mission.srv import snapshot, snapshotRequest
from tool_angle.srv import tool_angle, tool_angleRequest

pic_pos = \
[[11.5333, 27.6935, 14.078700000000001, 179.948, 10.215, -0.04],
[11.119, 11.5573, 14.078700000000001, -155.677, 9.338, 4.16],
[11.119, 45.8423, 15.5243, 162.071, 8.982, -2.503],
[20.0209, 29.7892, 13.6829, -179.401, 20.484, 0.484],
[-1.6163, 27.2584, 10.5365, 178.176, -5.075, -0.821],
[11.2913, 30.077499999999997, 3.8148000000000004, 176.897, 9.752, -0.733],
[11.2913, 48.3532, 0.1746, 147.166, 8.127, -5.457],
[11.2913, 14.063300000000002, -1.8908999999999998, -136.398, 7.255, 6.574],
[7.5134, 26.818099999999998, -2.06, 179.442, -22.966, -0.352],
[20.6853, 26.818099999999998, 0.048799999999999996, 179.502, 41.557, -0.951]]


class Arm_status(enum.IntEnum):
    Idle = 1
    Isbusy = 2

class State(enum.IntEnum):
    move2pic = 0
    take_pic = 1
    move2objup = 2
    move2obj = 3
    move2binup = 4
    move2placeup = 5
    place = 6
    finish = 7
    get_objinfo = 8
    placeup = 9
    move2bin_middleup = 10

class EasyCATest:
    def __init__(self):
        self.arm_move = False
        self.monitor_suc = False
        self.state = State.move2pic
        self.pic_pos = np.array(pic_pos)
        self.pic_pos_indx = 0
        self.target_obj = []
        self.place_pos_left = np.array([])##
        self.place_pos_right = np.array([])##
        self.dis_trans = np.mat(np.identity(4))

    def hand_eye_client(self, req):
        rospy.wait_for_service('/robot/eye_trans2base')
        try:
            ez_ca = rospy.ServiceProxy('/robot/eye_trans2base', eye2base)
            res = ez_ca(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def get_pcd_client(self, req):
        rospy.wait_for_service('/get_pcd')
        try:
            get_pcd = rospy.ServiceProxy('/get_pcd', save_pcd)
            res = get_pcd(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def CA_client(self, req):
        rospy.wait_for_service('/robot/_CA')
        try:
            ca = rospy.ServiceProxy('/robot/_CA', collision_avoid)
            res = ca(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def get_obj_client(self, req):
        rospy.wait_for_service('/AlignPointCloud')
        try:
            get_obj = rospy.ServiceProxy('/AlignPointCloud', snapshot)
            res = get_obj(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def tool_client(self, req):
        rospy.wait_for_service('/tool/tool_angle')
        try:
            tool = rospy.ServiceProxy('/tool/tool_angle', tool_angle)
            res = tool(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def Mission_Trigger(self):
        if self.arm_move == True and robot_ctr.get_robot_motion_state() == Arm_status.Isbusy:
            self.arm_move = False
        # if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        if robot_ctr.get_robot_motion_state() == Arm_status.Idle and self.arm_move == False:
            if self.state == State.move2pic:
                pos = self.pic_pos[self.pic_pos_indx]
                self.pic_pos_indx += 1
                # position = [pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]]
                pos[1] -= 3
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pos)
                self.state = State.take_pic
                self.arm_move = True

            elif self.state == State.take_pic:
                time.sleep(0.2)
                req = eye2baseRequest()
                req.ini_pose = np.array(np.identity(4)).reshape(-1)
                trans = self.hand_eye_client(req).tar_pose
                req = save_pcdRequest()
                req.curr_trans = np.array(trans)
                req.name = 'mdfk'                               
                if self.pic_pos_indx < len(self.pic_pos):
                    self.state = State.move2pic
                    req.save_mix = False
                else:
                    self.state = State.get_objinfo
                    req.save_mix = True
                self.get_pcd_client(req)

            elif self.state == State.get_objinfo:
                res = snapshotRequest()
                res.call = 0
                req = self.get_obj_client(res)
                # req = eye2baseRequest()
                # req.ini_pose = self.target_obj ##
                # trans = self.hand_eye_client(req).tar_pose
                self.state = State.move2objup

            elif self.state == State.move2objup:
                req = collision_avoid()
                req.ini_pose = self.target_obj ####
                req.limit = 2
                req.dis = 5
                res = self.CA_client(req)
                pose = res.tar_pose
                self.dis_trans = res.dis_trans
                self.suc_angle = res.suc_angle
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                req = tool_angleRequest()
                req.angle = self.suc_angle
                res = self.tool_client(req)
                self.state = State.move2obj
                self.arm_move = True


            elif self.state == State.move2obj:
                req = collision_avoid()
                req.ini_pose = self.target_obj ####
                req.limit = 2
                req.dis = 0
                res = self.CA_client(req)
                pose = res.tar_pose
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2binup
                self.monitor_suc = True
                self.arm_move = True

            elif self.state == State.move2binup:
                pose = [15,15,10,180,0,0]
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                req = tool_angleRequest()
                req.angle = 0
                res = self.tool_client(req)
                self.state = State.move2placeup
                self.arm_move = True

            elif self.state == State.move2placeup:
                if True:
                    pose = self.place_pos_left ##
                else:
                    pose = self.place_pos_right ##
                trans = tf.transformations.euler_matrix(radians(pose[3]), radians(pose[4]), radians(pose[5]), axes='sxyz')
                trans = np.mat(trans) * self.dis_trans
                pose[3:] = [degrees(abc) for abc in tf.transformations.euler_from_matrix(trans, axes='sxyz')]
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.place
                self.arm_move = True

            elif self.state == State.place:
                relat_pos = [0,0,-20,0,0,0]
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_RelPTPCmd(relat_pos)
                self.state = State.placeup
                self.arm_move = True

            elif self.state == State.placeup:
                relat_pos = [0,0,20,0,0,0]
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_RelPTPCmd(relat_pos)
                self.state = State.move2bin_middleup
                self.arm_move = True

            elif self.state == State.move2bin_middleup:
                pos = [11.5, 24, 14, 180, 10, 0]
                robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2objup
                self.arm_move = True

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
    robot_ctr.Set_tool_number(10)
    # tool_result = robot_ctr.Define_tool(1,tool_coor)
    robot_ctr.Set_operation_mode(1)
    robot_ctr.Set_override_ratio(100)
    poses = []
    strtage = EasyCATest()
    while strtage.state != State.finish and not rospy.is_shutdown():
        strtage.Mission_Trigger()
        time.sleep(0.1)

