# robot-simulation-v1.1

## robot_description
```
1. This package defines the urdf description of panda robot.

2. Use the following command to generate a urdf file from xacro file.
    xacro panda.xacro > panda.urdf

3. Use the following command to generate a sdf file from urdf file.
    gz sdf -p panda.urdf > panda.sdf

4. Use the following command to test the rightness of the robot model.
    ros2 launch robot_description test_panda_description.launch.py

5. (Note) urdf/panda_hw_interface.xacro
    It does not used in this package, but will be used in other packages.
```
