# denso_robot_ros

**Robot Model:** vs060

**Repo description:** Includes modification to the denso_robot_ros repo to add movement to robot. Some of the sample codes given in this tutorial were tested in the denso robot interface. A franka gripper were added to the denso arm in order to enable pick and place tasks.

**Added Projects:**

- simple_trajectory: Running
- ik_trajectory_exec: Running
- movegroup_cpp_interface: Running
- movegroup_py_interface: Running
- realsense_gazebo_plugin: Running
- perception_pipeline: Failing
- pick_place: Running

**How to run:**

To launch robot only:

    roslaunch denso_robot_bringup vs060_bringup.launch 

To launch robot with franka hand with realsense_plugin:

    roslaunch denso_robot_bringup vs060_hand_bringup.launch

To launch movegroup_cpp_interface:

    roslaunch denso_robot_bringup vs060_bringup.launch
    roslaunch movegroup_cpp_interface movegroup_cpp_interface.launch

To launch movegroup_py_interface:

    roslaunch denso_robot_bringup vs060_bringup.launch
    roslaunch movegroup_py_interface movegroup_oy_interface.launch

To run the simple trajectory:

    roslaunch denso_robot_bringup vs060_hand_bringup.launch
    rosrun simple_trajetory simple_trajectory

To launch pick and place:

    roslaunch denso_robot_bringup vs060_hand_bringup.launch
    rosrun pick_place pick_place_node

To add two tables side by side, and two cans over one of the tables:
    
    roslaunch denso_robot_bringup vs060_hand_bringup.launch
    rosrun denso_robot_gazebo spawn_objects











