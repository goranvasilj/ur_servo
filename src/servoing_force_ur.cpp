#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdio.h>

sensor_msgs::JointState joints;
geometry_msgs::WrenchStamped force_torque;

void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
	joints = *msg;
}

void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
	force_torque = *msg;
}
double sign (double x)
{
	if (x<0)
		return -1;
	if (x==0) return 0;
	return 1;
}
double forcex=0,forcey=0,forcez=0;
geometry_msgs::TwistStamped calculate_reference(double desired_force,
		double desired_velocity) {
	geometry_msgs::TwistStamped ref;
	double desired_vector[3] = { 0, 0, 1 };

	double force_gain = 0.1;
	double force_error[3];
	double force_x0 = 0;
	double force_y0 = 0;
	double force_z0 = 9;

	forcex=forcex*0.8+force_torque.wrench.force.x*0.2;
	forcey=forcey*0.8+force_torque.wrench.force.y*0.2;
	forcez=forcez*0.8+force_torque.wrench.force.z*0.2;

	force_error[0] = (-desired_vector[0]) * desired_force
			- (forcex-force_x0);
	force_error[1] = (-desired_vector[1]) * desired_force
			- (forcey-force_y0) ;
	force_error[2] = (-desired_vector[2]) * desired_force
			- (forcez-force_z0);
//	std::cout<<force_error[0]<< " "<< force_error[1]<<" "<<force_error[2]<<std::endl;

	double dx=0,dy=0,dz=0;
	if (force_gain * force_error[0]> 0)
	{
		dx=force_gain * force_error[0];
	}
	if (force_gain * force_error[1]> 0)
	{
		dy=force_gain * force_error[1];
	}
	if (force_gain * force_error[2]> 0)
	{
		dz=force_gain * force_error[2];
	}
	if (dx>2) dx=2;
	if (dy>2) dy=2;
	if (dz>2) dz=2;

	ref.header.stamp = ros::Time::now();
	ref.twist.linear.x=desired_velocity*desired_vector[0]*(1 - dx);
	ref.twist.linear.y=desired_velocity*desired_vector[1]*(1 - dy);
	ref.twist.linear.z=desired_velocity*desired_vector[2]*(1 - dz);

	if (fabs(ref.twist.linear.x) > desired_velocity*1.01) ref.twist.linear.x=0;
	if (fabs(ref.twist.linear.y) > desired_velocity*1.01) ref.twist.linear.y=0;
	if (fabs(ref.twist.linear.z) > desired_velocity*1.01) ref.twist.linear.z=0;

	double tx=force_torque.wrench.torque.x, ty=force_torque.wrench.torque.y, tz=force_torque.wrench.torque.z;
	if (fabs(force_torque.wrench.torque.x)<0.2)
	{
		tx=0;
	}
	if (fabs(force_torque.wrench.torque.y)<0.2)
	{
		ty=0;
	}
	if (fabs(force_torque.wrench.torque.z)<0.4)
	{
		tz=0;
	}
	double torque_gain=0.1;
	ref.twist.angular.x = -tx * torque_gain;
	ref.twist.angular.y = -ty * torque_gain;
	ref.twist.angular.z = -tz * torque_gain*0;

	printf("dx dy dz %.2f %.2f %.2f   %.2f  forceerror[2]  %.2f   %.2f %.2f %.2f\n",dx,dy,dz,forcez-force_z0,force_error[2],tx,ty,tz);
	//std::cout<<dx <<" "<<dy<<" "<<dz<<" "<<tx <<" "<<ty<< " "<<tz<<<<std::endl;

	if (fabs(ref.twist.angular.x) > desired_velocity) ref.twist.angular.x=desired_velocity*5 * sign(ref.twist.angular.x);
	if (fabs(ref.twist.angular.y) > desired_velocity) ref.twist.angular.y=desired_velocity*5 * sign(ref.twist.angular.y);
	if (fabs(ref.twist.angular.z) > desired_velocity) ref.twist.angular.z=desired_velocity*5 * sign(ref.twist.angular.z);

	return ref;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "servoing_force_ur");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	int refresh_rate;
	std::string joint_states_topic, force_torque_topic,
			joint_velocity_cmd_topic;
	std::string dynamixel_service;
	double desired_force, desired_velocity;
	nh_ns.param("joint_states_topic", joint_states_topic,
			(std::string) "/joint_states");
	nh_ns.param("force_torque_topic", force_torque_topic,
			(std::string) "/wrench");
	nh_ns.param("joint_desired_command", joint_velocity_cmd_topic,
			(std::string) "/servo_server/delta_twist_cmds");
	nh_ns.param("desired_force", desired_force, 5.0);
	nh_ns.param("desired_velocity", desired_velocity, 0.01);

	ros::Subscriber joints_subscriber = nh.subscribe(joint_states_topic, 1000,
			joints_callback);
	ros::Subscriber force_torque_subscriber = nh.subscribe(force_torque_topic,
			1000, force_torque_callback);

	geometry_msgs::TwistStamped reference;
	ros::Publisher velocity_reference_publisher = nh.advertise<
			geometry_msgs::TwistStamped>(joint_velocity_cmd_topic, 1000);

	ros::Rate loop_rate(500);
	while (ros::ok()) {
		reference=calculate_reference(desired_force,desired_velocity);
		velocity_reference_publisher.publish(reference);
		ros::spinOnce();
		loop_rate.sleep();

	}
}

