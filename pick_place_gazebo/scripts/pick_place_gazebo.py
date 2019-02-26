#!/usr/bin/env python

import move_group_interface
import simple_trajectory_interface
import rospy


def main():
  try:
    print("Initializing node... ")
    rospy.init_node("pick_place_gazebo")

    print ("============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ...")
    raw_input()
    movearm = move_group_interface.MoveGroupPyInterface()

    arm_traj = simple_trajectory_interface.RobotArm("vs060_hand")

    # movearm.go_to_joint_state()

    print ( "============ Press `Enter` to execute a movement using a pose goal ...")
    raw_input()
    movearm.go_to_pose_goal()

    positions = {
        'left':  [-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
        'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
    }

    rospy.on_shutdown(arm_traj.stop)
    # Command Current Joint Positions first
    # current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    # traj.add_point(current_angles, 0.0)

    # p1 = positions[limb]
    # traj.add_point(p1, 7.0)
    # traj.add_point([x * 0.75 for x in p1], 9.0)
    # traj.add_point([x * 1.25 for x in p1], 12.0)
    # traj.start()
    # traj.wait(15.0)

    print ( "============ Program complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()