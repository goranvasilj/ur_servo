#ifndef ROBOT_CPP
#define ROBOT_CPP
#include <ros/ros.h>  
#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/JointConstraint.h>
//#include <moveit/move_group/capability_names.h>
//#include <moveit/move_group_interface/move_group.h>
#include<trajectory_msgs/JointTrajectory.h>
#include<trajectory_msgs/JointTrajectoryPoint.h>
//#include <moveit/planning_interface/planning_interface.h>

#include <string>
#include "ros/ros.h"
#include <moveit_msgs/MoveGroupActionGoal.h>
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include<moveit/planning_interface/planning_interface.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/simple_action_client.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>


double jointInfo[6];
double jointRef[6];
ros::Publisher pub;
//actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *actionClient;
void MoveTrajectory(int n)
{

	ROS_INFO("Trajectory");
	double *point;
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.push_back( "shoulder_pan_joint");
	goal.trajectory.joint_names.push_back("shoulder_lift_joint");
	goal.trajectory.joint_names.push_back("elbow_joint");
	goal.trajectory.joint_names.push_back("wrist_1_joint");
	goal.trajectory.joint_names.push_back("wrist_2_joint");
	goal.trajectory.joint_names.push_back("wrist_3_joint");	
	for (int i=1;i<=n;i++)
	{
		trajectory_msgs::JointTrajectoryPoint p;
		for (int j=0;j<6;j++)
		{
			p.positions.push_back(jointInfo[j]+(jointRef[j]-jointInfo[j])*i/n);
		}
		ros::Duration duration(i*0.1);
		p.time_from_start = duration;	
		goal.trajectory.points.push_back(p);					

	}
	pub.publish(goal);
}


void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	sensor_msgs::JointState js = *msg;
	for (int i=0;i<6;i++)
	{
		if (js.name[i]=="shoulder_pan_joint")
		{	
			jointInfo[0] = js.position[i];
		}
		if (js.name[i]=="shoulder_lift_joint")
		{	
			jointInfo[1] = js.position[i];
		}
		if (js.name[i]=="elbow_joint")
		{	
			jointInfo[2] = js.position[i];
		}
		if (js.name[i]=="wrist_1_joint")
		{	
			jointInfo[3] = js.position[i];
		}
		if (js.name[i]=="wrist_2_joint")
		{	
			jointInfo[4] = js.position[i];
		}
		if (js.name[i]=="wrist_3_joint")
		{	
			jointInfo[5] = js.position[i];
		}

			
	}	
//	ROS_INFO("I heard: [%f %f %f %f %f %f]", jointInfo[0], jointInfo[1], jointInfo[2], jointInfo[3], jointInfo[4], jointInfo[5]);
}
void jointsReferenceCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	sensor_msgs::JointState js = *msg;
	jointRef[0] = js.position[0];
	jointRef[1] = js.position[1];
	jointRef[2] = js.position[2];
	jointRef[3] = js.position[3];
	jointRef[4] = js.position[4];
	jointRef[5] = js.position[5];
	ROS_INFO("I heard ref:  [%f %f %f %f %f %f]", jointRef[0], jointRef[1], jointRef[2], jointRef[3], jointRef[4], jointRef[5]);
	ROS_INFO("I heard state: [%f %f %f %f %f %f]", jointInfo[0], jointInfo[1], jointInfo[2], jointInfo[3], jointInfo[4], jointInfo[5]);	
	MoveTrajectory(100);
}
int main(int argc, char **argv)
{
	ROS_INFO("Start");
  ros::init(argc, argv, "move_trajectory");
  
  ros::NodeHandle n;

  pub =n.advertise<trajectory_msgs::JointTrajectory>("eff_joint_traj_controller/command", 10);;
  ros::Subscriber sub = n.subscribe("/joint_states", 10, jointsCallback);

  ros::Subscriber sub1 = n.subscribe("/joint_reference", 10, jointsReferenceCallback);

  ros::Rate loop_rate(500);

  //actionClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(n,"eff_joint_traj_controller",true);// "follow_joint_trajectory",true);
  //actionClient->waitForServer();

  int count = 0;
  while (ros::ok())
  {


    ros::spinOnce();

    loop_rate.sleep();
  }
	ROS_INFO("Finish");

  return 0;
}


#endif
