#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;

class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("vs060/arm_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("joint_1");
    goal.trajectory.joint_names.push_back("joint_2");
    goal.trajectory.joint_names.push_back("joint_3");
    goal.trajectory.joint_names.push_back("joint_4");
    goal.trajectory.joint_names.push_back("joint_5");
    goal.trajectory.joint_names.push_back("joint_6");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(2);

    double current_pos[6];
    get_current_joint_angles(current_pos);
    // ROS_INFO("current angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f",current_pos[0],current_pos[1],current_pos[2],current_pos[3],current_pos[4],current_pos[5]) ;

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = current_pos[0];
    goal.trajectory.points[ind].positions[1] = current_pos[1];
    goal.trajectory.points[ind].positions[2] = current_pos[2];
    goal.trajectory.points[ind].positions[3] = current_pos[3]; 
    goal.trajectory.points[ind].positions[4] = current_pos[4];
    goal.trajectory.points[ind].positions[5] = current_pos[5];

    //Velocities
    goal.trajectory.points[ind].velocities.resize(6);

    for (size_t j = 0; j < 6; ++j)
    {

      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(6);
    goal.trajectory.points[ind].positions[0] = current_pos[0]-0.5235;
    goal.trajectory.points[ind].positions[1] = current_pos[1];
    goal.trajectory.points[ind].positions[2] = current_pos[2];
    goal.trajectory.points[ind].positions[3] = current_pos[3]; 
    goal.trajectory.points[ind].positions[4] = current_pos[4];
    goal.trajectory.points[ind].positions[5] = current_pos[5];
    // Velocities
    goal.trajectory.points[ind].velocities.resize(6);
    for (size_t j = 0; j < 6; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

  //figure out where the arm is now
  void get_current_joint_angles(double current_pos[6])
  {
    int i;

    //get a single message from the topic 'arm_controller/state'
    sensor_msgs::JointState::ConstPtr joint_states =
        ros::topic::waitForMessage<sensor_msgs::JointState>("vs060/joint_states");

    //extract the joint angles from it
    for (i = 0; i <  6 ; i++)
    {
      current_pos[i] = joint_states->position[i];//    actual.positions[i];
    }
  }
};

class RobotHand
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;
  const static int n_joints = 2;

public:
  //! Initialize the action client and wait for action server to come up
  RobotHand() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("vs060_hand/hand_controller/follow_joint_trajectory", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotHand()
  {
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal handExtensionTrajectory(float finger1_pos, float finger2_pos )
  {
    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;
    

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("panda_finger_joint1");
    goal.trajectory.joint_names.push_back("panda_finger_joint2");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(n_joints);

    double current_pos[n_joints];
    get_current_joint_pos(current_pos);
    ROS_INFO("current position: %0.3f %0.3f",current_pos[0],current_pos[1]) ;

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(n_joints);
    goal.trajectory.points[ind].positions[0] = current_pos[0];
    goal.trajectory.points[ind].positions[1] = current_pos[1];


    //Velocities
    goal.trajectory.points[ind].velocities.resize(n_joints);

    for (size_t j = 0; j < n_joints; ++j)
    {

      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // Second trajectory point
    // Positions
    ind += 1;
    goal.trajectory.points[ind].positions.resize(n_joints);
    goal.trajectory.points[ind].positions[0] = finger1_pos;
    goal.trajectory.points[ind].positions[1] = finger2_pos;

    ROS_INFO("goal position: %0.3f %0.3f",goal.trajectory.points[ind].positions[0],goal.trajectory.points[ind].positions[1]) ;

    // Velocities
    goal.trajectory.points[ind].velocities.resize(n_joints);
    for (size_t j = 0; j < n_joints; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 2 seconds after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    //we are done; return the goal
    return goal;
  }

  control_msgs::FollowJointTrajectoryGoal openHand()
  {
    return handExtensionTrajectory(0.04, 0.04);
  }

  control_msgs::FollowJointTrajectoryGoal closeHand()
  {
    return handExtensionTrajectory(0.0, 0.0);
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

  //figure out where the hand is now
  void get_current_joint_pos(double current_pos[2])
  {
    int i;

    //get a single message from the topic 'arm_controller/state'
    sensor_msgs::JointState::ConstPtr joint_states =
        ros::topic::waitForMessage<sensor_msgs::JointState>("vs060_hand/joint_states");

    //extract the joint angles from it
    for (i = 0; i < n_joints ; i++)
    {
      current_pos[i] = joint_states->position[i];//    actual.positions[i];
    }
  }
};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory");

  // RobotArm arm;
  // // Start the trajectory
  // arm.startTrajectory(arm.armExtensionTrajectory());
  // // Wait for trajectory completion
  // while(!arm.getState().isDone() && ros::ok())
  // {
  //   usleep(50000);
  // }

  RobotHand hand;
  // Start the trajectory
  hand.startTrajectory(hand.openHand());
  // Wait for trajectory completion
  while(!hand.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }

  sleep(1);

  hand.startTrajectory(hand.closeHand());
  // Wait for trajectory completion
  while(!hand.getState().isDone() && ros::ok())
  {
    usleep(50000);
  }

  return 0;
}