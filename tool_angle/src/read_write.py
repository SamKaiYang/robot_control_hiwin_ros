#!/usr/bin/env python3
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Read and Write Example      *********
#
#
# Available DXL model on this example : All models using Protocol 1.0
# This example is tested with a DXL MX-28, and an USB2DYNAMIXEL
# Be sure that DXL MX properties are already set as %% ID : 1 / Baudnum : 34 (Baudrate : 57600)
#

import os
import time
import rospy
import ConfigParser
from dynamixel_sdk import *                    # Uses Dynamixel SDK library
from tool_angle.srv import ax_cali, tool_angle, ax_caliResponse, tool_angleResponse

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1023            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 5                # Dynamixel moving status threshold

class ToolAxis:
    def __init__(self):
        # Initialize self.PortHandler instance
        # Set the port path
        # Get methods and members of self.PortHandlerLinux or self.PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize self.PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1self.PacketHandler or Protocol2self.PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            quit()

        self.__tool_angle = rospy.Service('tool/tool_angle',
                tool_angle,
                self.__move_ax
        )
        self.__eye2base_server = rospy.Service('tool/ax_cali',
                ax_cali,
                self.__ax_calibration
        )
        self.__ini_pos = 0.
        self.__end_pos = 0.
        self.__tra_ini_pos = 0.
        self.__tar_end_pos = -90.
        self.__get_param()

    def test(self):
        dxl_goal_position = 10
        # Enable Dynamixel Torque
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
        while abs(self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0] - dxl_goal_position) > DXL_MOVING_STATUS_THRESHOLD:
            print(self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0])
            time.sleep(0.1)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()

    def __get_param(self):
        curr_path = os.path.dirname(os.path.abspath(__file__))
        config = ConfigParser.ConfigParser()
        path = curr_path + '\..\config\ax_param.ini'
        config.read(path)
        self.__ini_pos = float(config.get("Pos", "ini_pos"))
        self.__end_pos = float(config.get("Pos", "end_pos"))

    def __set_param(self):
        curr_path = os.path.dirname(os.path.abspath(__file__))
        config = ConfigParser.ConfigParser()
        path = curr_path + '\..\config\ax_param.ini'
        config.read(path)
        config.set("Pos", "ini_pos", str(self.__ini_pos))
        config.set("Pos", "end_pos", str(self.__end_pos))
        config.write(open(curr_path + '/img_trans.ini', 'wb'))

    def __ax_calibration(self, req):
        if 'dis' in str(req.cmd):
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        elif 'ini' in str(req.cmd):
            self.__ini_pos = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0]
            self.__set_param()
        elif 'end' in str(req.cmd):
            self.__end_pos = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)[0]
            self.__set_param()
        return True       

    def __map_angle(self, tar_angle):
        dis_ax = self.__end_pos - self.__ini_pos
        dis_tar = self.__tar_end_pos - self.__tra_ini_pos
        return (tar_angle - self.__tra_ini_pos) * dis_ax / dis_tar + self.__ini_pos 

    def __move_ax(self, req):
        dxl_goal_position = self.__map_angle(req.angle)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position)
        return True

if __name__ == "__main__":
    rospy.init_node('tool_axis')
    worker = ToolAxis()
    worker.test()
    rospy.spin()
