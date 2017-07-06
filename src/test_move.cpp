#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

class RobotArm
{
  typedef actionlib::SimpleActionClient< control_msgs::FollowJointTrajectoryAction > TrajClient;
private:
  // Action client for the joint trajectory action
  // used to trigger the arm movement action
  TrajClient* traj_client_;
  ros::NodeHandle nh_, priv_nh_;
  std::vector<std::string> joint_names_;
  unsigned int num_joints_;
public:
  //! Initialize the action client and wait for action server to come up
  RobotArm(const ros::NodeHandle &nh = ros::NodeHandle(), const ros::NodeHandle &priv_nh = ros::NodeHandle("~"))
    : nh_(nh), priv_nh_(priv_nh), num_joints_(6)
  {
    joint_names_.resize(num_joints_);
    joint_names_[0] = "shoulder_pan_joint";
    joint_names_[1] = "shoulder_lift_joint";
    joint_names_[2] = "elbow_joint";
    joint_names_[3] = "wrist_1_joint";
    joint_names_[4] = "wrist_2_joint";
    joint_names_[5] = "wrist_3_joint";

    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient(nh_, std::string("vel_based_pos_traj_controller/follow_joint_trajectory"), true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the follow_joint_trajectory server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    traj_client_->cancelAllGoals();
    delete traj_client_;
  }

  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
  {
    // When to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now();
    traj_client_->sendGoal(goal);
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */
  control_msgs::FollowJointTrajectoryGoal simpleTrajectory(double dir_offset)
  {
    //sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(std::string("joint_states"), nh_);

    //our goal variable
    control_msgs::FollowJointTrajectoryGoal goal;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names = joint_names_;

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    double start_times[2] = {0.0, 1.0};
    double joint_offset[2] = {0.0, dir_offset*0.02};
    double start_joints[6] = {1.1582108332119967, -1.1427507742263503, 1.158238412906511, -3.0567715694075828, -1.1923775513899795, 0.3263623701627276};
    // Trajectory points
    for (size_t ind = 0; ind < 1; ++ind)
    {
      goal.trajectory.points[ind].positions.resize(6);
      goal.trajectory.points[ind].velocities.resize(6);
      for (size_t j = 0; j < num_joints_; ++j)
      {
        // Positions
        goal.trajectory.points[ind].positions[j] = start_joints[j] + joint_offset[ind+1];
        // Velocities
        goal.trajectory.points[ind].velocities[j] = 0.0;
      }
      // To be reached 1 second after starting along the trajectory
      goal.trajectory.points[ind].time_from_start = ros::Duration(start_times[ind+1]);
    }

    //we are done; return the goal
    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }

};

int main(int argc, char** argv)
{
  // Init the ROS node
  ros::init(argc, argv, "simple_trajectory_player");

  RobotArm arm;
  double dir_offset = 1.0;
  ros::Rate r(0.01); // 1 hz
  while (ros::ok())
  {
    // Start the trajectory
    arm.startTrajectory(arm.simpleTrajectory(dir_offset));
    // Wait for trajectory completion
//    while(!arm.getState().isDone() && ros::ok())
//    {
//      ros::Duration(0.05).sleep();
//      ros::spinOnce();
//    }
    r.sleep();
    ros::spinOnce();
    dir_offset = dir_offset * -1;
  }
  return 0;
}
