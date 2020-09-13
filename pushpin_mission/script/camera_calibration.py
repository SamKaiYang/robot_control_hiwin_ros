#!/usr/bin/env python
import os
import time
import numpy as np
import rospy
import tf
import ConfigParser
import enum
import argparse
#import Hiwin_RT605_Socket_test_andy as ArmTask
from control_node import HiwinRobotInterface
from pushpin_mission.srv import hand_eye_calibration, hand_eye_calibrationRequest
from geometry_msgs.msg import Transform
from math import radians, degrees, pi
from std_msgs.msg import String, Bool
# --------------------------

# -----IMPORT cfg-----   BX = -28.7 BY = 31.8  BZ = -22.65
# from dynamic_reconfigure.server import Server
# from ROS_Socket.cfg import TutorialsConfig

# [[-0.03159894 -0.9994751  -0.0071438   0.08560622]
#  [ 0.9994982  -0.0316138   0.0019764   0.00585968]
#  [-0.0022012  -0.00707777  0.99997253 -0.11048413]
#  [ 0.          0.          0.          1.        ]]


pic_pos = [
           [0.0, 368.0, 293.5, -180.0, 0.0, 90.0],
[169.087, 367.999, 293.499, 179.999, 0.0, 89.999],
[-124.949, 367.999, 293.499, -179.999, 0.0, 89.999],
[-124.95, 389.112, 293.5, -179.999, 0.0, 89.999],
[-6.375, 343.099, 293.499, -179.999, 0.0, 114.776],
[11.587, 345.349, 293.5, -179.999, 0.0, 141.454],
[43.537, 350.037, 293.499, 178.254, 15.194, 173.366],
[43.537, 491.112, 293.499, 178.254, 15.194, 173.366],
[-67.059, 549.197, 293.109, 178.246, 15.183, 173.348],
[-67.059, 475.135, 293.109, 178.246, 15.183, 43.696],
[-126.309, 514.613, 290.709, 178.246, 15.183, 43.696],
[47.54, 512.325, 290.71, 178.246, 15.183, 10.671],
[127.938, 500.858, 221.335, 174.706, 29.131, 76.06],
[-113.189, 465.175, 238.726, 153.091, 17.557, 68.415],
[-19.107, 484.406, 222.025, 162.894, 14.341, 16.293],
[-14.25, 255.246, 159.015, 178.03, -17.123, 89.891],
[-14.25, 255.246, 93.877, 178.03, -17.123, 89.891],
[-14.195, 431.346, 78.277, -179.687, 13.316, 89.095],
[-14.195, 431.346, 167.19, -179.746, 16.908, 89.079],
[94.141, 504.729, 167.19, -179.746, 16.908, 89.079],
[140.191, 504.729, 167.19, -179.746, 16.908, 89.079],
[-63.208, 386.342, 167.19, 158.774, 2.216, 88.169]]

class Arm_status(enum.IntEnum):
    Idle = 1
    Isbusy = 2

class State(enum.IntEnum):
    move = 0
    take_pic = 1
    finish = 2

class CameraCalib:
    def __init__(self):
        self.arm_move = False
        self.state = State.move
        self.pos = np.array(pic_pos)

    def get_curr_pos(self):
        pose = robot_ctr.Get_current_position()
        # a = radians(pose[4]) + pi
        # b = -1 * (pi/2 + radians(pose[3]))
        # c = radians(pose[5]) + pi/2
        a = radians(pose[3])
        b = radians(pose[4])
        c = radians(pose[5])
        quaternion = tf.transformations.quaternion_from_euler(a, b, c, 'sxyz')
        # print('rpy: ', pose.x, ' ',pose.y, ' ',pose.z, ' ',pose.roll, ' ',pose.pitch, ' ',pose.yaw)
        # print('abc: ', degrees(a), degrees(b), degrees(c))
        # R = tf.transformations.quaternion_matrix(quaternion)
        # print(R)
        # print('------------------------------------------------------------------------')
        res = np.array([])
        res = np.append(res, pose[0]/1000)
        res = np.append(res, pose[1]/1000)
        res = np.append(res, pose[2]/1000)
        res = np.append(res, quaternion)
        return res

    def hand_eye_client(self, req):
        rospy.wait_for_service('/camera/hand_eye_calibration')
        try:
            hand_eye = rospy.ServiceProxy('/camera/hand_eye_calibration', hand_eye_calibration)
            res = hand_eye(req)
            return res
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def Mission_Trigger(self):
        if self.arm_move == True and robot_ctr.get_robot_motion_state() == Arm_status.Isbusy:
            self.arm_move = False
        # if Arm_state_flag == Arm_status.Idle and Sent_data_flag == 1:
        if robot_ctr.get_robot_motion_state() == Arm_status.Idle and self.arm_move == False:
            if self.state == State.move: 
                print('fuckin')
                pos = self.pos[0]

                positon = [pos[0]/10, pos[1]/10, pos[2]/10, pos[3], pos[4], pos[5]]
                # ArmTask.point_data(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
                # ArmTask.Arm_Mode(2,1,0,30,2)  #action,ra,grip,vel,both
                robot_ctr.Set_override_ratio(20)
                robot_ctr.Step_AbsPTPCmd(positon)
                self.pos = np.delete(self.pos, 0, 0)
                self.state = State.take_pic
                self.arm_move = True
                print(pos)
                
            elif self.state == State.take_pic:
                time.sleep(0.2)
                print('fuck_in_again')
                pose = self.get_curr_pos()
                req = hand_eye_calibrationRequest()
                if len(self.pos) == 0:
                    req.cmd = 'hello'
                else:
                    req.cmd = 'hi'
                req.end_trans.translation.x = pose[0]
                req.end_trans.translation.y = pose[1]
                req.end_trans.translation.z = pose[2]
                req.end_trans.rotation.x    = pose[3]
                req.end_trans.rotation.y    = pose[4]
                req.end_trans.rotation.z    = pose[5]
                req.end_trans.rotation.w    = pose[6]
                res = self.hand_eye_client(req)
                if res.is_done:
                    self.state = State.finish
                    # ArmTask.point_data(0, 36.8, 11.35, -90, 0, 0)
                    # ArmTask.Arm_Mode(2,1,0,10,2)
                    positon = [0.0,36.8,11.35,-180,0,90]
                    robot_ctr.Set_override_ratio(20)
                    robot_ctr.Step_AbsPTPCmd(positon)
                    trans_mat = np.array(res.end2cam_trans)
                    trans_mat = trans_mat.reshape(4,4)
                    print('##################################################################')
                    print(trans_mat)
                    print('##################################################################')
                    config = ConfigParser.ConfigParser()
                    config.optionxform = str  #reference: http://docs.python.org/library/configparser.html
                    curr_path = os.path.dirname(os.path.abspath(__file__))
                    # config.read(['img_trans_pinto.ini', curr_path])
                    config.read(curr_path + '/img_trans.ini')
                    
                    config.set("Section_A", "Key_1_1", str(trans_mat[0][0]))
                    config.set("Section_A", "Key_1_2", str(trans_mat[0][1]))
                    config.set("Section_A", "Key_1_3", str(trans_mat[0][2]))
                    config.set("Section_A", "Key_2_1", str(trans_mat[1][0]))
                    config.set("Section_A", "Key_2_2", str(trans_mat[1][1]))
                    config.set("Section_A", "Key_2_3", str(trans_mat[1][2]))
                    config.set("Section_A", "Key_3_1", str(trans_mat[2][0]))
                    config.set("Section_A", "Key_3_2", str(trans_mat[2][1]))
                    config.set("Section_A", "Key_3_3", str(trans_mat[2][2]))

                    config.write(open(curr_path + '/img_trans.ini', 'wb'))

                else:
                    self.state = State.move

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
    rospy.init_node('calibration_'+robot_name,
                    log_level=log_level,
                    disable_signals=True)
    if rospy.get_param("use_sim_time", False):
        rospy.logwarn("use_sim_time is set!!!")

    robot_ctr = HiwinRobotInterface(robot_ip=robot_ip, connection_level=control_mode,name=robot_name)
    robot_ctr.connect()


    robot_ctr.Set_operation_mode(0)
    # set tool & base coor
    tool_coor = [0,0,0,0,0,0]
    base_coor = [0,0,0,0,0,0]
    robot_ctr.Set_base_number(1)
    base_result = robot_ctr.Define_base(1,base_coor)
    robot_ctr.Set_tool_number(1)
    tool_result = robot_ctr.Define_tool(1,tool_coor)
    #ArmTask.run()
    robot_ctr.Set_operation_mode(1)
    #ArmTask.Arm_Mode(4,1,0,10,2)
    robot_ctr.Set_override_ratio(20)
    strtage = CameraCalib()
    
    while strtage.state != State.finish and not rospy.is_shutdown():
        strtage.Mission_Trigger()
        time.sleep(0.1)
    
    rospy.spin()


    # a = np.array([[ 0.01638994, -0.98366769,  0.17924633,  0.04688393],
    #               [ 0.99983572,  0.01473635, -0.01055294, -0.03196167],
    #               [ 0.00773915,  0.17938985,  0.98374762,  0.00542407],
    #               [ 0.        ,  0.        ,  0.        ,  1.        ]])

    # c = np.array([[ 0.04030787, -0.98179438,  0.18562077,  0.04309001],
    #               [ 0.99911025,  0.03729612, -0.01969007, -0.02738011],
    #               [ 0.01240867,  0.18624928,  0.98242416,  0.00389264],
    #               [ 0.        ,  0.        ,  0.        ,  1.        ]])
             
    # d = np.array([[ 0.00873828, -0.98539554,  0.17005668,  0.04796343],
    #               [ 0.99990635,  0.00681918, -0.01186591, -0.02881461],
    #               [ 0.01053297,  0.17014444,  0.98536284, -0.01808258],
    #               [ 0.        ,  0.        ,  0.        ,  1.        ]])
             
    # e = np.array([[ 0.01261697, -0.98530859,  0.17031672,  0.04767782],
    #               [ 0.99986393,  0.01062171, -0.01262113, -0.02836312],
    #               [ 0.01062665,  0.17045279,  0.98530854, -0.01813825],
    #               [ 0.        ,  0.        ,  0.        ,  1.        ]])
             
    # f = np.array([[ 0.01570163, -0.9852051 ,  0.17065862,  0.04736422],
    #               [ 0.99982031,  0.0136573 , -0.0131465 , -0.02802579],
    #               [ 0.01062126,  0.17083438,  0.98524251, -0.01882511],
    #               [ 0.        ,  0.        ,  0.        ,  1.        ]])

    # b = np.array([[ 3.92108452e-03, -9.83089646e-01,  1.83082966e-01,  4.73223894e-02],
    #               [ 9.99718856e-01, -4.27615074e-04, -2.37070994e-02, -2.21137451e-02],
    #               [ 2.33844930e-02,  1.83124451e-01,  9.82811580e-01,  9.60093652e-03],
    #               [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

#avggg
# [[ 0.0162793  -0.98407682  0.17649701  0.04671697]
#  [ 0.99970924  0.01378384 -0.01526394 -0.02777651]
#  [ 0.0125522   0.1766992   0.98414954 -0.00602138]
#  [ 0.          0.          0.          1.        ]]