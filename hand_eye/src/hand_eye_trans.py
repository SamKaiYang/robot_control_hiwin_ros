#!/usr/bin/env python3
import os
import numpy as np
import rospy
import tf
import ConfigParser
from math import radians, degrees, pi
from control_node.msg import robot_info
from hand_eye.srv import eye2base, eye2baseResponse

class HandEyeTrans:
    def __init__(self):
        self._img_pose = np.zeros(6)
        self._base_pose = np.zeros(6)
        self._curr_pose = np.zeros(6)
        self._tool_coor = np.zeros(6)
        self._base_coor = np.zeros(6)
        self._base_tool_trans =  np.mat(np.identity(4))
        self._rbase_base_trans = np.mat(np.identity(4))
        self._rtool_tool_trans = np.mat(np.identity(4))
        self._hand_eye_trans =   np.mat(np.identity(4))
        self._rtool_eye_trans, self._camera_mat = self.__get_camera_param()
        self.__trans_sub = rospy.Subscriber(
                'robot/curr_info',
                robot_info,
                self.__robot_info_callback,
                queue_size=1
        )
        self.__eye2base_server = rospy.Service('robot/eye2base',
                eye2base,
                self.__eye2base_transform
        )
        self.__pis2base_server = rospy.Service('robot/pix2base',
                eye2base,
                self.__pix2base_transform
        )
        self.__eye_trans2base_server = rospy.Service('robot/eye_trans2base',
                eye2base,
                self.__eye_trans2base_transform
        )
        self.__pis2base_server = rospy.Service('robot/down_cam2base',
                eye2base,
                self.__down_pix2base_transform
        )
        self.__pis2base_server = rospy.Service('robot/fix_cam2base',
                eye2base,
                self.__fix_pix2base_transform
        )
        self.down_cam_trans = np.mat([[0, -1, 0, 0.153],
                                      [1, 0, 0, 0.305],
                                      [0, 0, 1, -0.89],
                                      [0, 0, 0, 1]])

        self.down_cam_obj_z = 0.6

    def __get_camera_param(self):
        curr_path = os.path.dirname(os.path.abspath(__file__))
        config = ConfigParser.ConfigParser()
        path = curr_path + '\..\config\img_trans.ini'
        print(path)
        config.read(path)
        a00 = float(config.get("External", "Key_1_1"))
        a01 = float(config.get("External", "Key_1_2"))
        a02 = float(config.get("External", "Key_1_3"))
        a03 = float(config.get("External", "Key_1_4"))
        a10 = float(config.get("External", "Key_2_1"))
        a11 = float(config.get("External", "Key_2_2"))
        a12 = float(config.get("External", "Key_2_3"))
        a13 = float(config.get("External", "Key_2_4"))
        a20 = float(config.get("External", "Key_3_1"))
        a21 = float(config.get("External", "Key_3_2"))
        a22 = float(config.get("External", "Key_3_3"))
        a23 = float(config.get("External", "Key_3_4"))

        b00 = float(config.get("Internal", "Key_1_1"))
        b01 = float(config.get("Internal", "Key_1_2"))
        b02 = float(config.get("Internal", "Key_1_3"))
        b10 = float(config.get("Internal", "Key_2_1"))
        b11 = float(config.get("Internal", "Key_2_2"))
        b12 = float(config.get("Internal", "Key_2_3"))
        b20 = float(config.get("Internal", "Key_3_1"))
        b21 = float(config.get("Internal", "Key_3_2"))
        b22 = float(config.get("Internal", "Key_3_3"))
        
        Ex = np.mat([[a00, a01, a02, a03],
                     [a10, a11, a12, a13],
                     [a20, a21, a22, a23],
                     [0,   0,   0,   1]])
        In = np.mat([[b00, b01, b02],
                     [b10, b11, b12],
                     [b20, b21, b22]])
        print(Ex,In)
        return Ex, In

    def __robot_info_callback(self, msg):
        self._curr_pose = np.array(msg.curr_pose)
        self._tool_coor = np.array(msg.tool_coor)
        # self._base_coor = np.array(msg.base_coor)

    def __get_robot_trans(self):
        abc = [radians(i) for i in self._curr_pose[3:]]
        self._base_tool_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')[0:3, 0:3]
        self._base_tool_trans[0:3, 3:] = np.mat([i/100 for i in self._curr_pose[:3]]).reshape(3, 1)

        # abc = [radians(i) for i in self._base_coor[3:]]
        # self._rbase_base_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')
        # self._rbase_base_trans[0:3, 3:] = np.mat([i/100 for i in self._base_coor[:3]]).reshape(3, 1)

        abc = [radians(i) for i in self._tool_coor[3:]]
        self._rtool_tool_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')[0:3, 0:3]
        self._rtool_tool_trans[0:3, 3:] = np.mat([i/100 for i in self._tool_coor[:3]]).reshape(3, 1)

    def __eye2base_transform(self, req):
        self.__get_robot_trans()
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
        eye_obj_trans[:3] = np.multiply(eye_obj_trans[:3], 0.01)
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.tar_pose = np.array(np.multiply(result[:3], 100)).reshape(-1)
        return res

    def __eye_trans2base_transform(self, req):
        self.__get_robot_trans()
        eye_obj_trans = np.mat(req.ini_pose).reshape(4, 4)
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.tar_pose = np.array(result).reshape(-1)
        print('result\\n', result)
        print('result\\n', res.tar_pose)
        return res
        

    def __pix2base_transform(self, req):
        self.__get_robot_trans()
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
        eye_obj_trans[2] = eye_obj_trans[2] * 0.01
        eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
        eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
        result = self._base_tool_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.tar_pose = np.array(np.multiply(result[:3], 100)).reshape(-1)
        return res

    def __down_pix2base_transform(self, req):
        self.__get_robot_trans()
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), [self.down_cam_obj_z, 1])).reshape(4, 1)
        eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
        eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
        result = self.down_cam_trans * eye_obj_trans
        res = eye2baseResponse()
        res.tar_pose = np.array(np.multiply(result[:3], 100)).reshape(-1)
        return res

    def __fix_pix2base_transform(self, req):
        # self.__get_robot_trans()
        positon =  [11.3440, 36.4321, 11.23, 179.994, 10.002, -0.488]
        abc = [radians(i) for i in positon[3:]]
        base_cam_trans = np.mat(np.identity(4))
        base_cam_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')[0:3, 0:3]
        base_cam_trans[0:3, 3:] = np.mat([i/100 for i in positon[:3]]).reshape(3, 1)
        eye_obj_trans = np.mat(np.append(np.array(req.ini_pose), 1)).reshape(4, 1)
        eye_obj_trans[2] = eye_obj_trans[2] * 0.01
        eye_obj_trans[:2] = (eye_obj_trans[:2] - self._camera_mat[:2, 2:]) * eye_obj_trans[2]
        eye_obj_trans[:2] = np.multiply(eye_obj_trans[:2], [[1/self._camera_mat[0, 0]], [1/self._camera_mat[1, 1]]])
        result = base_cam_trans * np.linalg.inv(self._rtool_tool_trans) * self._rtool_eye_trans * eye_obj_trans
        res = eye2baseResponse()
        res.tar_pose = np.array(np.multiply(result[:3], 100)).reshape(-1)
        return res

if __name__ == "__main__":
    rospy.init_node('hand_eye_trans')
    worker = HandEyeTrans()
    rospy.spin()



