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
    It does not work in this package, but will work in other packages.
```

## robot_gazebo_env
```
1. This is not a ros2 package, but a gazebo model package.

2. panda/panda.sdf
    It is copied from "robot_description/urdf", and then be modified a little.
    (1) "<?xml version="1.0"?>" is added to the first line in panda.sdf.
    (2) String "robot_description" is replaced by "panda" in panda.sdf.

3. Use the following command to open the gazebo simulation environment.
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robot-simulation-v1.1/src/robot_gazebo_env" >> ~/.bashrc
    gazebo ~/robot-simulation-v1.1/src/robot_gazebo_env/world/panda.world
```

## robot_info
```
1. This package defines the basic information of a robot. 
    A shared library "librobot_info.so" is generated, and will be used as a basic
    common library in other packages.
```

## shared_memory
```
1. This package defines some shared memory operations, which can realize communication
    with different process.
    Usually, it is a bridge between ros2 controller manager and real robot.
    In this simulation project, it is a bridge between ros2 controller manager and robot in gazebo.

2. This package will generate a shared library "libshared_memory.so", which can be used in other packages.
```

## robot_gazebo_plugin
```
1. This package realized a robot gazebo control plugin, which can communicate with shared memory.
    The plugin is named "librobot_gazebo_plugin.so".

2. To use the plugin, add following environment variable to your ~/.bashrc.
    echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/robot-simulation-v1.1/install/robot_gazebo_plugin/lib" >> ~/.bashrc

3. Add following line to "robot_gazebo_env/world/panda.world" within panda model.
    <plugin name="robot_gazebo_control_plugin" filename="librobot_gazebo_plugin.so" />
```