import rospy
import enum
import time
import numpy as np
import tf
import copy
from math import degrees, radians, cos
from control_node import HiwinRobotInterface
from collision_avoidance.srv import collision_avoid, collision_avoidRequest
from hand_eye.srv import eye2base, eye2baseRequest
from hand_eye.srv import save_pcd, save_pcdRequest
from avoidance_mission.srv import snapshot, snapshotRequest, snapshotResponse
from tool_angle.srv import tool_angle, tool_angleRequest
RATIO = 50
PTPSPEED = 15
PTPSPEED_SLOW = 10

pic_pos = \
[[11., 27., 14., 179.948, 10.215, -0.04],
[11., 11., 14., -155.677, 9.338, 4.16],
[11., 45., 15., 162.071, 8.982, -2.503],
[20., 29., 13., -179.401, 20.484, 0.484],
[-1., 27., 10., 178.176, -5.075, -0.821],
[11., 30., 3., 176.897, 9.752, -0.733],
[11., 48., 0., 147.166, 8.127, -5.457],
[11., 14., -1., -136.398, 7.255, 6.574],
[7., 26., -2., 179.442, -22.966, -0.352],
[20., 26., 0., 179.502, 41.557, -0.951]]


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
    move2placeup1 = 11
    pick_obj = 12

class EasyCATest:
    def __init__(self):
        self.arm_move = False
        self.monitor_suc = False
        self.state = State.move2pic
        self.pic_pos = np.array(pic_pos)
        self.pic_pos_indx = 0
        self.target_obj = []
        self.place_pos_left = np.array([-2.6175, -15.7, -29, 180, 0, 0])##
        self.place_pos_right = np.array([-2.6236, -26.8867, -29, 180, 0, 0])##
        self.dis_trans = np.mat(np.identity(4))
        self.right_side = False
        self.stop_flg = False

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

    def check_side(self, trans):
        if abs(np.dot(np.array(trans[:3, 2]).reshape(-1), [0, 0, 1])) > 0.7:
            if trans[2,2] > 0:
                transform = tf.transformations.euler_matrix(0, radians(180), 0, axes='sxyz')
                trans = trans * transform
                r_side = True
            else:
                r_side = False
        else:
            vec_cen2obj = trans[:2, 3].reshape(-1) - np.array([0.2,0.3])
            vec_obj = trans[:2, 2]
            if np.dot(vec_cen2obj, vec_obj) / (np.linalg.norm(vec_cen2obj) * np.linalg.norm(vec_obj)) > 0:
                transform = tf.transformations.euler_matrix(0, radians(180), 0, axes='sxyz')
                trans = trans * transform
                r_side = True
            else:
                r_side = False
        self.target_obj = trans
        return r_side
        


    def Mission_Trigger(self):
        if self.arm_move == True and robot_ctr.get_robot_motion_state() == Arm_status.Isbusy:
            self.arm_move = False
        # if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        if self.monitor_suc == True:
            robot_inputs_state = robot_ctr.Get_current_robot_inputs()
            if robot_inputs_state[0] == True:
                robot_ctr.Stop_motion()  #That is, it is sucked and started to place
                time.sleep(0.2)
                self.monitor_suc = False
                self.state = State.pick_obj
        dig_inputs = robot_ctr.Get_current_digital_inputs()
        if self.stop_flg == True:
            if dig_inputs[2] == False
                self.stop_flg = False
            self.state = State.move2pic
        elif dig_inputs[2] == True:
            self.stop_flg = True
            return

        if robot_ctr.get_robot_motion_state() == Arm_status.Idle and self.arm_move == False:
            if self.state == State.move2pic:
                pos = self.pic_pos[self.pic_pos_indx]
                # self.pic_pos_indx += 1
                position = [pos[0], pos[1]-3, pos[2], pos[3], pos[4], pos[5]]
                # pos[1] -= 3
                # robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(position)
                self.state = State.get_objinfo
                self.arm_move = True
                # time.sleep(10)

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
                time.sleep(0.2)
                req = snapshotRequest()
                req.call = 0
                res = self.get_obj_client(req)
                # res = snapshotResponse()
                # res.doit = True
                # res.type = 0
                # t = tf.transformations.rotation_matrix(radians(90), [0, 1, 0], point=None)
                # res.trans = np.array([1,0,0,0,0,1,0,0,0,0,1,0.59,0,0,0,1])
                # t = np.array([0.746, -0.665, -0.011, 0.172, -0.663, -0.745, 0.066, -0.089, -0.053, -0.042, -0.997, 0.58, 0,0,0,1])
                # t = [0.108, 0.977, -0.180, -0.089, -0.993, 0.103, -0.040, -0.183, -0.020, 0.183, 0.982,0.58, 0,0,0,1]
                # t = [-0.040, -0.998, 0.033, -0.047, -0.999, 0.040, 0.012, -0.034, -0.013, -0.033, -0.999,  0.58, 0,0,0,1]
                # t = [0.906, -0.388, -0.168, -0.243, 0.334, 0.900, -0.278, -0.029, 0.259, 0.196, 0.945, 0.58, 0, 0, 0, 1]
                # t = [0.660, 0.750, -0.022, -0.282, -0.750, 0.660, 0.006, 0.146, 0.019, 0.012, 0.999, 0.58,0, 0, 0, 1]
                # res.trans = np.array(t).reshape(-1)
                # res.trans[11] = 0.58
                if res.doit == True:
                    trans = np.mat(np.asarray(res.trans)).reshape(4,4)
                    trans[2,3] = 0.59
                    if res.type == 1:
                        trans = np.array(trans).reshape(-1)                        
                    elif res.type == 2: # y 90
                        # pre_trans = np.mat([[1., 0, 0, 0],
                        #                     [0,  1, 0, 0],
                        #                     [0,  0, 1, 0],
                        #                     [0,  0, 0, 1]])
                        pre_trans = tf.transformations.euler_matrix(0, radians(90), 0, axes='sxyz')
                        trans = trans * pre_trans
                        # trans = pre_trans * trans
                        trans = np.array(trans).reshape(-1)
                    elif res.type == 3: # -90
                        # pre_trans = np.mat([[1., 0, 0, 0],
                        #                     [0,  1, 0, 0],
                        #                     [0,  0, 1, 0],
                        #                     [0,  0, 0, 1]])
                        pre_trans = tf.transformations.euler_matrix(0, radians(-90), 0, axes='sxyz')
                        trans = trans * pre_trans
                        # trans = pre_trans * trans
                        print('fucktrans', trans)
                        trans = np.array(trans).reshape(-1)

                    req = eye2baseRequest()
                    req.ini_pose = trans ##
                    self.target_obj = self.hand_eye_client(req).tar_pose
                    self.target_obj = np.mat(self.target_obj).reshape(4,4)
                    self.right_side = self.check_side(self.target_obj)
                     
                    x, y, z = np.array(np.multiply(self.target_obj[0:3, 3:], 100)).reshape(-1)
                    a, b, c = [degrees(abc) for abc in tf.transformations.euler_from_matrix(self.target_obj, axes='sxyz')]
                    self.target_obj = [x, y, z, a, b, c]
                    print('self.target_obj\n ', self.target_obj)
                    self.state = State.move2objup
                    self.pic_pos_indx = 0
                else:
                    # self.pic_pos_indx += 1
                    self.pic_pos_indx = self.pic_pos_indx  if self.pic_pos_indx < len(self.pic_pos) else 0
                    self.state = State.move2pic

            elif self.state == State.move2objup:
                req = collision_avoidRequest()
                req.ini_pose = np.array(self.target_obj).reshape(-1) ####
                req.limit = 1.5
                req.dis = 6
                res = self.CA_client(req)
                pose = np.array(res.tar_pose)
                self.dis_trans = np.mat(res.dis_trans).reshape(4,4)
                self.suc_angle = res.suc_angle
                # robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                req = tool_angleRequest()
                req.angle = self.suc_angle
                res = self.tool_client(req)
                self.state = State.move2obj
                self.arm_move = True


            elif self.state == State.move2obj:
                robot_ctr.Set_digital_output(1,True)
                req = collision_avoidRequest()
                req.ini_pose = np.array(self.target_obj).reshape(-1) ####
                req.limit = 2
                req.dis = -1
                res = self.CA_client(req)
                pose = np.array(res.tar_pose)
                robot_ctr.Set_ptp_speed(PTPSPEED_SLOW)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.pick_obj
                self.monitor_suc = True
                self.arm_move = True

            elif self.state == State.pick_obj:
                req = collision_avoidRequest()
                req.ini_pose = np.array(self.target_obj).reshape(-1) ####
                req.limit = 2
                req.dis = 6
                res = self.CA_client(req)
                pose = np.array(res.tar_pose)
                robot_ctr.Set_ptp_speed(PTPSPEED)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2binup
                self.arm_move = True

            elif self.state == State.move2binup:
                if self.monitor_suc == True:
                    time.sleep(0.3)
                    self.monitor_suc = False
                pose = [15,15,10,180,0,0]
                # robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2placeup
                self.arm_move = True

            

            elif self.state == State.move2placeup:
                req = tool_angleRequest()
                req.angle = 0
                res = self.tool_client(req)
                robot_inputs_state = robot_ctr.Get_current_robot_inputs()
                if robot_inputs_state[0] == False:
                    robot_ctr.Set_digital_output(1,False)
                    self.state = State.move2pic
                    return
                pose = [7.7,-18,5.5714,180,0,0]
                # robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2placeup1
                self.arm_move = True

            elif self.state == State.move2placeup1:
                pose = [7.7,-18,-22,180,0,0]
                trans = tf.transformations.euler_matrix(radians(pose[3]), radians(pose[4]), radians(pose[5]), axes='sxyz')
                trans = np.mat(trans) * self.dis_trans
                pose[3:] = [degrees(abc) for abc in tf.transformations.euler_from_matrix(trans, axes='sxyz')]
                print('pose:\,n', pose)
                # robot_ctr.Set_ptp_speed(10)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.place
                self.arm_move = True

            elif self.state == State.place:
                if self.right_side:
                    self.place_pos_right[0] += 6
                    if self.place_pos_right[0] > 18:
                        self.place_pos_right[0] = 18
                        self.place_pos_right[2] += 1.5
                    pose = copy.deepcopy(self.place_pos_right)  ##
                else:
                    self.place_pos_left[0] += 6
                    if self.place_pos_left[0] > 18:
                        self.place_pos_left[0] = 18
                        self.place_pos_left[2] += 1.5
                    pose = copy.deepcopy(self.place_pos_left) ##
                print('pose:\,n', pose)
                trans = tf.transformations.euler_matrix(radians(pose[3]), radians(pose[4]), radians(pose[5]), axes='sxyz')
                print(trans)
                print(self.dis_trans)
                trans = np.mat(trans) * self.dis_trans
                pose[3:] = [degrees(abc) for abc in tf.transformations.euler_from_matrix(trans, axes='sxyz')]
                print('pose:\,n', pose)
                robot_ctr.Set_ptp_speed(PTPSPEED_SLOW)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.placeup
                self.arm_move = True
                ##
            
            elif self.state == State.placeup:
                robot_ctr.Set_digital_output(1,False)
                pose = [7.7,-20,5.5714,180,0,0]
                robot_ctr.Set_ptp_speed(PTPSPEED)
                robot_ctr.Step_AbsPTPCmd(pose)
                self.state = State.move2pic
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
    robot_ctr.Set_override_ratio(RATIO)
    robot_ctr.Set_ptp_speed(PTPSPEED)
    poses = []
    strtage = EasyCATest()
    while strtage.state != State.finish and not rospy.is_shutdown():
        strtage.Mission_Trigger()
        time.sleep(0.1)

# ('self.target_obj\n ', [15.476806363754292, 59.16117557076886, -28.04519572390219, 134.60712652596033, -39.17113006977304, -31.40485516079049])