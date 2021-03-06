#!/usr/bin/env python3
import os
import numpy as np
# import rospy
import transformations as tf
import copy
import configparser
from math import radians, degrees, pi, acos, atan2
# from control_node.msg import robot_info
# from collision_avoidance.srv import collision_avoid, collision_avoidResponse

class Collision_avoidRequest:
    def __init__(self):
        self.ini_pose = []
        self.limit = 0.
        self.dis = 0.

class Collision_avoidResponse:
    def __init__(self):
        self.tar_pose = []
        self.dis_trans = []
        self.suc_angle = 0.

class CollisionAvoidance:
    def __init__(self, limit_array):
        self._hx, self._lx, self._hy, self._ly, self._lz = limit_array
        self.limit = 0.1
        self.suction_len = 0.043
        self.single_trans_angle = 5
        self._curr_pose = np.zeros(6)
        self._tool_coor = np.array([0,0,180,0,0,0])
        self._base_coor = np.zeros(6)
        self._base_tool_trans =  np.mat(np.identity(4))
        self._rbase_base_trans = np.mat(np.identity(4))
        self._rtool_tool_trans = np.mat([[1,0,0,0,0,1,0,0,0,0,1,0.18,0,0,0,1]]).reshape(4,4)
        self._tar_trans = np.mat(np.identity(4))
        self._ini_trans = np.mat(np.identity(4))
        # self.__trans_sub = rospy.Subscriber(
        #         'robot/curr_info',
        #         robot_info,
        #         self.__robot_info_callback,
        #         queue_size=1
        # )
        # self.__eye2base_server = rospy.Service('robot/easy_CA',
        #         collision_avoid,
        #         self.__easy_collision_avoid
        # )
        # self.__pis2base_server = rospy.Service('robot/_CA',
        #         collision_avoid,
        #         self.__complete_collision_avoid
        # )

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

    def _check_position(self, limit):
        xyz = [i for i in np.array(self._tar_trans[0:3, 3:]).reshape(-1)]
        limits = [self._hx-limit, self._lx+limit, self._hy-limit, self._ly+limit, self._lz]
        xyz[0] = limits[0] if xyz[0] > limits[0] else xyz[0]
        xyz[0] = limits[1] if xyz[0] < limits[1] else xyz[0]
        xyz[1] = limits[2] if xyz[1] > limits[2] else xyz[1]
        xyz[1] = limits[3] if xyz[1] < limits[3] else xyz[1]
        xyz[2] = limits[4] if xyz[2] < limits[4] else xyz[2]
        self._tar_trans[0:3, 3:] = np.mat(xyz).reshape(3, 1)

    def _check_collision(self):
        alarm = False
        base_rtool_trans = self._tar_trans * np.linalg.inv(self._rtool_tool_trans)
        # print(base_rtool_trans)
        xyz = [i for i in base_rtool_trans[0:3, 3:]]
        limits = [self._hx-self.limit, self._lx+self.limit, self._hy-self.limit, self._ly+self.limit]

        if xyz[0] > limits[0] or xyz[0] < limits[1]:
            direction = np.array(np.linalg.inv(self._tar_trans[0:3, 0:3])[0:3, 1:2]).reshape(-1) # get base Y axis in end trans
            angle = self.single_trans_angle if xyz[0] < limits[1] else -1 * self.single_trans_angle
            trans_mat = np.mat(tf.transformations.rotation_matrix(radians(angle), direction, point=None)) # use axis angle to get rotation
            self._tar_trans = self._tar_trans * trans_mat
            alarm = True

        if xyz[1] > limits[2] or xyz[1] < limits[3]:
            direction = np.array(np.linalg.inv(self._tar_trans[0:3, 0:3])[0:3, 0:1]).reshape(-1) # get base X axis in end trans
            angle = self.single_trans_angle if xyz[1] > limits[2] else -1 * self.single_trans_angle
            trans_mat = np.mat(tf.transformations.rotation_matrix(radians(angle), direction, point=None))
            self._tar_trans = self._tar_trans * trans_mat
            alarm = True
        return alarm

    def __easy_collision_avoid(self, req):
        self.__get_robot_trans()
        abc = [radians(i) for i in req.ini_pose[3:]]
        self._ini_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')[0:3, 0:3]
        self._ini_trans[0:3, 3:] = np.mat([i/100 for i in req.ini_pose[:3]]).reshape(3, 1)
        self._tar_trans = copy.deepcopy(self._ini_trans)
        print(self._tar_trans)
        limit = req.limit/100 if req.limit > 0 else 0
        self._check_position(limit if limit < 0.1 else 0.1)
        for i in range(10):
            if self._check_collision() is False:
                print('fuckk: ', i)
                break
        # make distance between target point and tool end base on tool Z axis
        mat = np.mat(np.identity(4))
        dis = req.dis/100 if req.dis is not None and req.dis > 0 else 0
        mat[2, 3] = -dis if dis < 0.1 else -0.1
        print(self._tar_trans)
        self._tar_trans = self._tar_trans * mat
        print(self._tar_trans)

        res = Collision_avoidResponse()
        x, y, z = np.array(np.multiply(self._tar_trans[0:3, 3:], 100)).reshape(-1)
        a, b, c = [degrees(abc) for abc in tf.transformations.euler_from_matrix(self._tar_trans, axes='sxyz')]
        res.tar_pose = [x, y, z, a, b, c]
        return res

    def complete_collision_avoid(self, req):
        '''
        1. get suction vector
        2. update end position (include the dis between obj and end point) defalt orientation is 180, 0, 0
        3. check position
        4. check collision
        5. caculate suc angle and Z axis rotation
        6. update end trans (without suction)
        7. caculate trans mat from end suction to obj
        '''
        self.__get_robot_trans()
        abc = [radians(i) for i in req.ini_pose[3:]]
        self._ini_trans[0:3, 0:3] = tf.transformations.euler_matrix(abc[0], abc[1], abc[2], axes='sxyz')[0:3, 0:3]
        self._ini_trans[0:3, 3:] = np.mat([i/100 for i in req.ini_pose[:3]]).reshape(3, 1)
        self._tar_trans = copy.deepcopy(self._ini_trans)
        limit = req.limit/100 if req.limit > 0 else 0
        self._check_position(limit if limit < 0.1 else 0.1)
        mat = np.mat(np.identity(4))
        dis = req.dis/100 if req.dis is not None and req.dis > 0 else 0
        mat[2, 3] = -dis if dis < 0.1 else -0.1
        mat[2, 3] -= self.suction_len
        # print('-2: \n', self._tar_trans)
        self._tar_trans = self._tar_trans * mat
        # print('-1: \n', self._tar_trans)
        # print('-0.5: \n', mat)
        self._tar_trans[0:3, 0:3] = tf.transformations.euler_matrix(radians(180), 0, 0, axes='sxyz')[0:3, 0:3]
        # print('0: \n', self._tar_trans)
        for i in range(10):
            if self._check_collision() is False:
                print('fuckk: ', i)
                break
        # print('1: \n', self._tar_trans)
        suc_angle = acos(np.dot(np.array(self._ini_trans[:3, 2:3]).reshape(-1), np.array(self._tar_trans[:3, 2:3]).reshape(-1))) 
        suc_angle = -1 * suc_angle if suc_angle < pi/2 else -1 * (pi - suc_angle)  # because ax_12 axis
        tool_obj_trans = np.linalg.inv(self._tar_trans) * self._ini_trans
        # z_proj = np.append(np.array(tool_obj_trans[2, :2]), 0.)
        # print(np.dot(z_proj, [1,0,0]))
        # tool_z_angle = acos(np.dot(z_proj, [1,0,0])+1e-8 / (np.linalg.norm(z_proj)+1e-8))
        tool_z_angle = atan2(-1 * tool_obj_trans[1, 2], -1 * tool_obj_trans[0,2])
        # print(tool_z_angle)
        self._tar_trans = self._tar_trans * np.mat(tf.transformations.rotation_matrix(tool_z_angle, [0, 0, 1], point=None))
        # print('2: \n', self._tar_trans)
        suc_trans = np.mat(tf.transformations.rotation_matrix(suc_angle, [0, 1, 0], point=None))
        # print('suc_trans: ', suc_trans)
        # print('self._ini_trans: \n', self._ini_trans)
        # dis_trans = self._ini_trans * np.linalg.inv(suc_trans) * np.linalg.inv(self._tar_trans)
        # base_obj_trans = self._tar_trans * suc_trans * dis_trans
        # print('base_obj_trans: \n', base_obj_trans)
        # base_end_trans = self._tar_trans * suc_trans
        # print('base_end_trans: \n', base_end_trans)
        dis_trans = np.linalg.inv(suc_trans) * np.linalg.inv(self._tar_trans) * self._ini_trans
        _, direc, _ = tf.transformations.rotation_from_matrix(dis_trans)
        if abs(np.dot(direc, [0, 0, 1])) < 0.99:
            print('FUCKNONON_________FUCK__________ONONONONONOFUFK')
            # print(direc)
        else:
            print('NiceN__________Nice_____________NNNice')
        mat = np.mat(np.identity(4))
        mat[2, 3] = self.suction_len
        # print('2.5: \n', self._tar_trans)
        # print('2.75: \n', mat)
        self._tar_trans = self._tar_trans * mat
        # print('3: \n', self._tar_trans)
        res = Collision_avoidResponse()
        x, y, z = np.array(np.multiply(self._tar_trans[0:3, 3:], 100)).reshape(-1)
        a, b, c = [degrees(abc) for abc in tf.transformations.euler_from_matrix(self._tar_trans, axes='sxyz')]
        res.tar_pose = [x, y, z, a, b, c]
        res.dis_trans = np.array(dis_trans).reshape(-1)
        res.suc_angle = degrees(suc_angle)
        return res

if __name__ == "__main__":
    # rospy.init_node('collision_avoidance')
    limit_array = [0.394, 0.006, 0.587, 0.001, -0.3]
    worker = CollisionAvoidance(limit_array)
    req = Collision_avoidRequest()
    req.ini_pose = [0, 0,   3,  180, 90, -25]
    req.limit = 2.5
    req.dis = 0
    res = worker.complete_collision_avoid(req)
    print(res.tar_pose)
    print(res.dis_trans)
    print(res.suc_angle)
    # rospy.spin()
