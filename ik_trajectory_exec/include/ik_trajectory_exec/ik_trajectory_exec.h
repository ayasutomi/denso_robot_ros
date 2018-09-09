#ifndef _IK_TRAJECTORY_EXEC_H
#define _IK_TRAJECTORY_EXEC_H

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <vector>

#define MAX_JOINT_VEL 2.0 //in radians/sec

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;
static const std::string ARM_IK_NAME = "/compute_ik";


class IKTrajectoryExecutor
{

  private:
    ros::NodeHandle node;
    ros::ServiceClient ik_client;
    ros::ServiceServer service;
    control_msgs::FollowJointTrajectoryGoal goal;
    moveit_msgs::GetPositionIK::Request ik_request;
    moveit_msgs::GetPositionIK::Response ik_response;
    TrajClient *action_client;

  public:
    IKTrajectoryExecutor();

    ~IKTrajectoryExecutor();

    //run inverse kinematics on a PoseStamped (7-dof pose
    //(position + quaternion orientation) + header specifying the
    //frame of the pose)
    //tries to stay close to double start_angles[7]
    //returns the solution angles in double solution[7]
    bool run_ik(geometry_msgs::PoseStamped pose, double start_angles[6],
                double solution[6], std::string link_name);

    //figure out where the arm is now
    void get_current_joint_angles(double current_angles[6]);

    //send a desired joint trajectory to the joint trajectory action
    //and wait for it to finish
    bool execute_joint_trajectory(std::vector<double *> joint_trajectory);

    //service function for execute_cartesian_ik_trajectory
    bool execute_cartesian_ik_trajectory(geometry_msgs::PoseStamped newposeStamped);
};

#endif