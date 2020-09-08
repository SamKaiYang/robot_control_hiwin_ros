## How to use the module
```python
from control_node import HiwinRobotInterface
```
### Making Python Module for Another Package
* catkin [user guide](http://docs.ros.org/api/catkin/html/user_guide/setup_dot_py.html) for setup

1. Uncomment following line in CMakeLists.txt
```cmake
catkin_python_setup()
```

2. Let structure of package like following:
```
-- package_name/
   |-- some_folder/
   |-- src/
       |-- package_name/
           |-- __init__.py
           |-- some_package/
           |-- some_module.py
   |-- CMakeLists.txt
   |-- package.xml
   |-- setup.py
```

3. Make a setup.<span></span>py in package
```python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['package_name'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```

4. Build the package
```bash
cd SOMEWORKSPACE
catkin_make
```


# HIWIN ROS-I Package
##  Run simulated MoveIt! environment without real robot
- Run the demo launch file with the command:

  `roslaunch hiwin_robot_moveit_config demo.launch manipulator_model:=<manipulator_model> gripper_model:=<gripper_model>`

  Note: 
     - substitute `<manipulator_model>` with the robot model (e.g. _ra605_710_gb_).
     - substitute `<gripper_model>` with the gripper model (e.g. _xeg_16_).

## Run package to control a real Robot and Gripper
If you already purchased a robotic arm and an electric gripper you can follow the following steps:

1. Connect the robotic arm to your network through an ethernet cable, make sure your computer and your robot are on the same domain (e.g. robot's ip is 192.168.0.1, computer's ip is 192.168.0.100).

1. Connect your gripper via USB to your computer and check in which COM port is the USB insert. (You can use [XEG-W1](https://www.hiwin.tw/support/ee/eg_software.aspx) software to check gripper's COM Port)

1. Load the control nodes for robot and gripper:

    `roslaunch hiwin_driver hiwin_robot_interface.launch manipulator_ip:=<manipulator_ip>`

    `roslaunch hiwin_driver hiwin_gripper_interface.launch gripper_model:=<gripper_model> gripper_com_port:=<gripper_com_port>`

    Note: 
     - substitute `<manipulator_ip>` with the IP address of the robotic arm.
     - substitute `<gripper_model>` with the gripper model (e.g. _xeg_16_).
     - substitute `<gripper_com_port>` with the gripper p (16 or 32).
     

1. Load the model, run MoveIt! and (optional) run the RVIZ simulated environment

    `roslaunch hiwin_robot_moveit_config start_planning.launch manipulator_model:=<manipulator_model> gripper_model:=<gripper_model>`

    Note: 
     - substitute `<manipulator_model>` with the robot model (default is _ra605_710_gb_)
     - substitute `<gripper_model>` with the gripper model (e.g. _xeg_16_).

### Licence

All the files included in this directory are under the BSD 3 Clause Licence. A copy of the licence is included in this folder.