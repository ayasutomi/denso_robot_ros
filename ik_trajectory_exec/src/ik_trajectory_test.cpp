#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <ik_trajectory_exec/ik_trajectory_exec.h>

int main(int argc, char **argv)
{
    //init the ROS node                                                                                      
    ros::init(argc, argv, "ik_trajectory_exec");
    ros::NodeHandle nh;

    // //Initialize joint angle goals to move arms to initial position                                          
    // RobotArm r_arm("r_arm");
    // RobotArm l_arm("l_arm");
    // double r_arm_pos[] = {-0.52, -0.5, -1.57, -1.0, 0.0, 0.0, 1.57};
    // double l_arm_pos[] = {0.7, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0};
    // ROS_INFO("Moving Arms to Initial Position");

    // // Move the arms to initial position                                                                     
    // r_arm.startTrajectory(r_arm_pos);
    // l_arm.startTrajectory(l_arm_pos);
    // // Wait for trajectory completion                                                                        
    // while (!r_arm.getState().isDone() && !l_arm.getState().isDone() && ros::ok())
    // {
    //     usleep(50000);
    // }

    // Calculate arm joint angles through Inverse Kinematics                                                 
    // and move end effector to grab position                                                                
    IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();
    geometry_msgs::PoseStamped newposeStamped;
    newposeStamped.header.frame_id = "/base_link";
    newposeStamped.pose.position.x = 0.5;
    newposeStamped.pose.position.y = 0.0;
    newposeStamped.pose.position.z = 0.5;
    newposeStamped.pose.orientation.x = 0.0;
    newposeStamped.pose.orientation.y = 0.0;
    newposeStamped.pose.orientation.z = 0.0;
    newposeStamped.pose.orientation.w = 1.0;
    ROS_INFO("Executing cartesian ik trajectory");
    ik_traj_exec.execute_cartesian_ik_trajectory(newposeStamped);

    ROS_INFO("IK Trajectory Execution Complete!");

    return 0;
}

