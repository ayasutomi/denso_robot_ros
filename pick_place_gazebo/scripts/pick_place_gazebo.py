#!/usr/bin/env python

import move_group_interface
import simple_trajectory_interface
import rospy
import time


def main():
  try:

    rospy.init_node('pick_place_gazebo', anonymous=True)

    movearm = move_group_interface.MoveGroupPyInterface()
    arm_traj = simple_trajectory_interface.RobotArm("vs060_hand")
    hand_traj = simple_trajectory_interface.RobotHand("vs060_hand")
    rospy.on_shutdown(arm_traj.stop)
    rospy.on_shutdown(hand_traj.stop)

    print ("============ Move group and simple action lib instantiated. Press `Enter` to continue ...")
    raw_input() 

    # movearm.go_to_joint_state()
    # movearm.go_to_pose_goal()


    current = arm_traj.get_current_joint_pos()
    print("Current Angles: " + str(current))
    # arm_traj.add_point(current[0:6], 0.0)

    # target = current[0:6]
    # target[0] -= 0.5235

    # arm_traj.add_point(target, 2.0)
    # arm_traj.start()
    # arm_traj.wait(15.0)

    hand_traj.openHand()

    time.sleep(2)

    hand_traj.closeHand()


    print ( "============ Program complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()