import rospy
from control_node import HiwinRobotInterface

if __name__ == "__main__":
    rospy.init_node('echo_curr_pose_node')
    robot_ctr = HiwinRobotInterface(robot_ip="192.168.0.1", connection_level=1,name="manipulator")
    robot_ctr.connect()


    robot_ctr.Set_operation_mode(0)
    # set tool & base coor
    tool_coor = [0,0,0,0,0,0]
    base_coor = [0,0,0,0,0,0]
    robot_ctr.Set_base_number(31)
    # base_result = robot_ctr.Define_base(1,base_coor)
    robot_ctr.Set_tool_number(15)
    # tool_result = robot_ctr.Define_tool(1,tool_coor)
    robot_ctr.Set_operation_mode(1)
    robot_ctr.Set_override_ratio(5)
    poses = []
    while True:
        key = raw_input('enter')
        if key == "q":
            break
        pose = robot_ctr.Get_current_position()
        poses.append(pose)

    for pose in poses:
        print '{},'.format(pose)
