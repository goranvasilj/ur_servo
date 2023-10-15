#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <control_msgs/JointJog.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <std_msgs/Int32.h>

sensor_msgs::JointState joints;
geometry_msgs::WrenchStamped force_torque;
control_msgs::JointJog servo_joint_reference, servo_joint_reference_old;
geometry_msgs::Twist servo_reference;
int received_reference1 = 0;
int received_reference2 = 0;
int force_limiting=0;
bool first=true;
control_msgs::JointJog old_servo_joint_reference;
ros::Time time_joint_reference;
ros::Time time_joint_reference_old;

void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
	joints = *msg;
}

void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
	force_torque = *msg;
}

void servo_reference_callback(const geometry_msgs::Twist::ConstPtr &msg) {
	received_reference1 = 1;
	received_reference2 = 0;
	servo_reference = *msg;
}
double sign(double x) {
	if (x < 0)
		return -1;
	if (x == 0)
		return 0;
	return 1;
}
void servo_joint_reference_callback(const control_msgs::JointJog::ConstPtr &msg) {
	received_reference1 = 0;
	received_reference2 = 1;
	if (first==false)
	{
		servo_joint_reference_old=servo_joint_reference;
		time_joint_reference_old=time_joint_reference;
	}
	servo_joint_reference = *msg;
	time_joint_reference=ros::Time::now();
	if (first==true)
	{
		for (int i=0;i<6;i++)
		{
			servo_joint_reference.velocities[i]=0;
		}
		servo_joint_reference_old=servo_joint_reference;
		time_joint_reference_old=time_joint_reference;
	}
	else
	{
		ros::Duration diff=time_joint_reference-time_joint_reference_old;
		double dt=diff.toSec();
		if (dt>0.2)
		{
			dt=0.002;
		}
		if (dt<0.001)
		{
			dt=0.001;
		}
		double max_acceleration=0.1;
		for (int i=0;i<6;i++)
		{
			double new_value=servo_joint_reference.velocities[i];
			double dx=servo_joint_reference.velocities[i]-servo_joint_reference_old.velocities[i];
			if (fabs(dx/dt)>max_acceleration)
			{
				new_value=servo_joint_reference_old.velocities[i]+sign(dx)*max_acceleration*dt;
			}
			if (fabs(new_value)<fabs(servo_joint_reference.velocities[i]))
			{
				servo_joint_reference.velocities[i]=new_value;
			}
		}
	}
	first=false;
}

double forcex = 0, forcey = 0, forcez = 0;
double tx = 0, ty = 0, tz = 0;
double force_x0 = 0;
double force_y0 = 0;
double force_z0 = 3;
int reset_force=0;
geometry_msgs::TwistStamped calculate_reference(double desired_force,
		double desired_velocity) {
	double dist_lin = sqrt(
			servo_reference.linear.z * servo_reference.linear.z
					+ servo_reference.linear.x * servo_reference.linear.x
					+ servo_reference.linear.y * servo_reference.linear.y);
	if ((servo_reference.linear.z) < 0.99 * dist_lin || fabs(servo_reference.linear.z)<0.0001 ) {
		geometry_msgs::TwistStamped ref;
		ref.twist=servo_reference;
		ref.header.stamp = ros::Time::now();
		reset_force=1;
		force_limiting=0;
		return ref;
	} else {


		geometry_msgs::TwistStamped ref;

		double desired_vector[3] = { 0, 0, 1 };
		if ((servo_reference.linear.z)<0)
		{
			desired_vector[2]=-1;
		}
		double force_gain = 0.05;//0.1      //total force controller gain is force_gain * desired_velocity
		double force_error[3];

		forcex = forcex * 0.8 + force_torque.wrench.force.x * 0.2;
		forcey = forcey * 0.8 + force_torque.wrench.force.y * 0.2;
		forcez = forcez * 0.8 + force_torque.wrench.force.z * 0.2;
		if (reset_force>0)
		{
			force_x0 = forcex;
			force_y0 = forcey;
			force_z0 = forcez;
			ref.twist.linear.x=0;
			ref.twist.linear.y=0;
			ref.twist.linear.z=0;
			ref.twist.angular.x=0;
			ref.twist.angular.y=0;
			ref.twist.angular.z=0;
			reset_force=reset_force+1;
			if (reset_force<1000)
			{
				return ref;
			}
		}
		reset_force=0;
		force_error[0] = (-desired_vector[0]) * desired_force
				- (forcex - force_x0);
		force_error[1] = (-desired_vector[1]) * desired_force
				- (forcey - force_y0);
		force_error[2] = (-desired_vector[2]) * desired_force
				- (forcez - force_z0);

//	std::cout<<force_error[0]<< " "<< force_error[1]<<" "<<force_error[2]<<std::endl;

		double dx = 0, dy = 0, dz = 0;
		if (force_gain * force_error[0] > 0) {
			dx = force_gain * force_error[0];
		}
		if (force_gain * force_error[1] > 0) {
			dy = force_gain * force_error[1];
		}
		if (force_gain * force_error[2] > 0) {
			dz = force_gain * force_error[2];
		}
		if (dx > 6)
			dx = 6;
		if (dy > 6)
			dy = 6;
		if (dx<3)
			dx=0;
		else
			dx=dx-3;
		if (dy<3)
			dy=0;
		else
			dy=dy-3;
		if (dz > 6)                 /// (dz-1)*desired_velocity  maximal speed in counter direction
			dz = 6;

		ref.header.stamp = ros::Time::now();
		ref.twist.linear.x = desired_velocity /* desired_vector[0] */* (1 - dx);
		ref.twist.linear.y = desired_velocity /* desired_vector[1] */* (1 - dy);
		ref.twist.linear.z = desired_velocity * desired_vector[2] * (1 - dz);
		static int force_limit_count=0;
		if (fabs(forcez - force_z0)>3)
		{
			force_limit_count++;
			if (force_limit_count>40)
			{
				force_limiting=1;
			}
		}
		else
		{
			force_limit_count=0;
		}

		if (fabs(ref.twist.linear.x) > desired_velocity * 1.01*5)
			ref.twist.linear.x = desired_velocity*5;
		if (fabs(ref.twist.linear.y) > desired_velocity * 1.01*5)
			ref.twist.linear.y = desired_velocity*5;
		if (fabs(ref.twist.linear.z) > desired_velocity * 1.01*5)
			ref.twist.linear.z = desired_velocity*5;

		tx = tx * 0.9 + 0.1 * force_torque.wrench.torque.x;
		ty = ty * 0.9 + 0.1 * force_torque.wrench.torque.y;
		tz = tz * 0.9 + 0.1 * force_torque.wrench.torque.z;
		if (fabs(tx)  < 0.2) {
			tx = 0;
		}
		else
		{
			force_limiting=1;
		}

		if (fabs(ty) < 0.2) {
			ty = 0;
		}
		else{
			force_limiting=1;
		}
		if (fabs(force_torque.wrench.torque.z) < 0.4) {
			tz = 0;
		}
		else
		{
			force_limiting=1;
		}
		std::cout<<"force limiting "<<force_limiting<<" error"<<force_error[2]<< "forcez forcez0 "<<forcez<<" " <<force_z0<<"tx ty "<< tx<<" "<<ty<<std::endl;

		double torque_gain = 0.02;
		ref.twist.angular.x = -tx * torque_gain;
		ref.twist.angular.y = -ty * torque_gain;
		ref.twist.angular.z = -tz * torque_gain * 0;
		printf("force %5.2f %5.2f %5.2f\n", forcex, forcey, forcez);
		printf(
				"dx dy dz %.2f %.2f %.2f   %.2f  forceerror[2]  %.2f   %.2f %.2f %.2f\n",
				dx, dy, dz, forcez - force_z0, force_error[2], tx, ty, tz);
		//std::cout<<dx <<" "<<dy<<" "<<dz<<" "<<tx <<" "<<ty<< " "<<tz<<<<std::endl;

		if (fabs(ref.twist.angular.x) > desired_velocity * 5)
			ref.twist.angular.x = desired_velocity * 5
					* sign(ref.twist.angular.x);
		if (fabs(ref.twist.angular.y) > desired_velocity * 5)
			ref.twist.angular.y = desired_velocity * 5
					* sign(ref.twist.angular.y);
		if (fabs(ref.twist.angular.z) > desired_velocity * 5)
			ref.twist.angular.z = desired_velocity * 5
					* sign(ref.twist.angular.z);
		return ref;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "servoing_force_ur");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	int refresh_rate;
	std::string joint_states_topic, force_torque_topic,
			twist_velocity_cmd_topic, joint_velocity_cmd_topic;;
	std::string dynamixel_service;
	double desired_force, desired_velocity;
	nh_ns.param("joint_states_topic", joint_states_topic,
			(std::string) "/joint_states");
	nh_ns.param("force_torque_topic", force_torque_topic,
			(std::string) "/wrench");


	nh_ns.param("twist_desired_command", twist_velocity_cmd_topic,
			(std::string) "/servo_server/delta_twist_cmds");

	nh_ns.param("joint_desired_command", joint_velocity_cmd_topic,
			(std::string) "/servo_server/delta_joint_cmds");

	nh_ns.param("desired_force", desired_force, 5.0);
	nh_ns.param("desired_velocity", desired_velocity, 0.02);

	ros::Subscriber joints_subscriber = nh.subscribe(joint_states_topic, 1000,
			joints_callback);
	ros::Subscriber servo_reference_subscriber = nh.subscribe(
			"/servo_reference", 1000, servo_reference_callback);
	ros::Subscriber servo_joint_reference_subscriber = nh.subscribe(
			"/servo_joint_reference", 1000, servo_joint_reference_callback);
	ros::Subscriber force_torque_subscriber = nh.subscribe(force_torque_topic,
			1000, force_torque_callback);

	geometry_msgs::TwistStamped reference;
	ros::Publisher velocity_reference_publisher = nh.advertise
			< geometry_msgs::TwistStamped > (twist_velocity_cmd_topic, 1000);

	ros::Publisher velocity_joint_reference_publisher = nh.advertise
			< control_msgs::JointJog > (joint_velocity_cmd_topic, 1000);


	ros::Publisher force_limiting_publisher = nh.advertise
			< std_msgs::Int32> ((std::string)"/servo_force_limiting", 1000);


/*

	joint_reference.joint_names.push_back("shoulder_pan_joint");
	joint_reference.joint_names.push_back("shoulder_lift_joint");
	joint_reference.joint_names.push_back("elbow_joint");
	joint_reference.joint_names.push_back("wrist_1_joint");
	joint_reference.joint_names.push_back("wrist_2_joint");
	joint_reference.joint_names.push_back("wrist_3_joint");
	joint_reference.velocities.push_back(0);
	joint_reference.velocities.push_back(0);
	joint_reference.velocities.push_back(0);
	joint_reference.velocities.push_back(0);
	joint_reference.velocities.push_back(0);
	joint_reference.velocities.push_back(0.02);

	joint_reference.duration=0.1;*/
	ros::Rate loop_rate(500);
	std_msgs::Int32 force_limiting_msg;
	while (ros::ok()) {
		force_limiting=0;
		reference = calculate_reference(desired_force, desired_velocity);
		reference.header.stamp= ros::Time::now();
		servo_joint_reference.header.stamp= ros::Time::now();
		std::cout << reference <<" received reference 1 "<<received_reference1<<" received reference 2 "<<received_reference2<< std::endl;
		std::cout<<servo_joint_reference<<std::endl;
		if (received_reference1 == 1)
		{
			velocity_reference_publisher.publish(reference);
		}
		else if (received_reference2==1)
		{
			velocity_joint_reference_publisher.publish(servo_joint_reference);
		}
		force_limiting_msg.data=force_limiting;
		force_limiting_publisher.publish(force_limiting_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
}

