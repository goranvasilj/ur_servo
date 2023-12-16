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
#include <std_msgs/Float32MultiArray.h>
#include <stdio.h>
#include <std_msgs/Int32.h>

sensor_msgs::JointState joints;
geometry_msgs::WrenchStamped force_torque;
control_msgs::JointJog servo_joint_reference, servo_joint_reference_old, servo_joint_reference_received;
geometry_msgs::Twist servo_reference;
int received_servo_reference_cartesian_space = 0;
int received_servo_reference_joint_space = 0;
int force_limiting = 0;
bool first = true;
control_msgs::JointJog old_servo_joint_reference;
ros::Time time_joint_reference;
ros::Time time_joint_reference_old, time_joint_reference_received;
std_msgs::Float32MultiArray tool_pose;
double max_acceleration = 0.2;
bool received_tool_pose = false;

//callback for receiving positions of individual joints
void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
	joints = *msg;
}

//callback for receiving 3 axis force and torque feedback of individual joints
void force_torque_callback(const geometry_msgs::WrenchStamped::ConstPtr &msg) {
	force_torque = *msg;
}

//callback for receiving tool pose in robot base coordinate frame
void tool_pose_callback(const std_msgs::Float32MultiArray::ConstPtr &msg) {
	tool_pose = *msg;
	received_tool_pose = true;
}

//receiving reference for servoing in cartesian space
void servo_reference_callback(const geometry_msgs::Twist::ConstPtr &msg) {
	received_servo_reference_cartesian_space = 1;
	received_servo_reference_joint_space = 0;
	servo_reference = *msg;
}

//function that calculates signum of a number
double sign(double x) {
	if (x < 0)
		return -1;
	if (x == 0)
		return 0;
	return 1;
}

//callback for receiving servoing commands for individual joint
void servo_joint_reference_callback(
		const control_msgs::JointJog::ConstPtr &msg) {

	received_servo_reference_cartesian_space = 0;
	received_servo_reference_joint_space = 1;

	servo_joint_reference_received = *msg;
	time_joint_reference_received = ros::Time::now();
}

//calculate roll angle in local coordinate frame, it corresponds to tilting of gripper left - right
double get_roll() {
	if (received_tool_pose == false)
		return 0;
	double yaw = atan2(tool_pose.data[4], tool_pose.data[0]);
	double pitch = atan2(-tool_pose.data[8],
			sqrt(
					tool_pose.data[9] * tool_pose.data[9]
							+ tool_pose.data[10] * tool_pose.data[10]));
	double roll = atan2(tool_pose.data[9], tool_pose.data[10]);
	printf("%.2f\n", roll);
	roll = roll - 3.14159265;
	if (roll < -3.14159265)
		roll += 2 * 3.14159265;
	printf("roll pitch yaw %.4f %.2f %.2f\n", roll, pitch, yaw);

	return roll;
}



double forcex = 0, forcey = 0, forcez = 0;
double tx = 0, ty = 0, tz = 0;
double force_x0 = 0;
double force_y0 = 0;
double force_z0 = 3;
int reset_force = 0;
//claculate reference for the servoing node
geometry_msgs::TwistStamped calculate_reference(double desired_force,
		double desired_velocity) {
	double roll = get_roll();
	double dist_lin = sqrt(
			servo_reference.linear.z * servo_reference.linear.z
					+ servo_reference.linear.x * servo_reference.linear.x
					+ servo_reference.linear.y * servo_reference.linear.y);


	//if not servoing down copy received values
	if ((servo_reference.linear.z) < 0.99 * dist_lin
			|| fabs(servo_reference.linear.z) < 0.0001) {
		geometry_msgs::TwistStamped ref;
		ref.twist = servo_reference;
		ref.header.stamp = ros::Time::now();
		reset_force = 1;
		force_limiting = 0;
		return ref;
	} else {
		//if servoing down limit maximal force
		geometry_msgs::TwistStamped ref;

		//vector for goint down;
		double desired_vector[3] = { 0, 0, 1 };
		if ((servo_reference.linear.z) < 0) {
			desired_vector[2] = -1;
		}

		double force_gain = 0.1; //0.1      //total force controller gain is force_gain * desired_velocity
		double force_error[3];

		//filter received force components
		forcex = forcex * 0.8 + force_torque.wrench.force.x * 0.2;
		forcey = forcey * 0.8 + force_torque.wrench.force.y * 0.2;
		forcez = forcez * 0.8 + force_torque.wrench.force.z * 0.2;

		//before sevoing down, measure average force and send values 0 for 2s
		if (reset_force > 0) {
			force_x0 = forcex;
			force_y0 = forcey;
			force_z0 = forcez;
			ref.twist.linear.x = 0;
			ref.twist.linear.y = 0;
			ref.twist.linear.z = 0;
			ref.twist.angular.x = 0;
			ref.twist.angular.y = 0;
			ref.twist.angular.z = 0;
			reset_force = reset_force + 1;
			if (reset_force < 1000) {
				return ref;
			}
		}

		//calculate error for each force component
		reset_force = 0;
		force_error[0] = (-desired_vector[0]) * desired_force
				- (forcex - force_x0);
		force_error[1] = (-desired_vector[1]) * desired_force
				- (forcey - force_y0);
		force_error[2] = (-desired_vector[2]) * desired_force
				- (forcez - force_z0);
		std::cout << "force z feedback" << (forcez - force_z0) << std::endl;
		std::cout << "force error" << force_error[0] << " " << force_error[1]
				<< " " << force_error[2] << std::endl;

		//mulitply error with gain
		double dx = 0, dy = 0, dz = 0;
		if (fabs(force_gain * force_error[0]) > 0) {
			dx = force_gain * force_error[0];
		}
		if (fabs(force_gain * force_error[1]) > 0) {
			dy = force_gain * force_error[1];
		}
		if (force_gain * force_error[2] > 0) {
			dz = force_gain * force_error[2];
		}


		//limit output
		if (fabs(dx) > 6)
			dx = 6 * sign(dx);
		if (fabs(dy) > 6)
			dy = sign(dy) * 6;

		//allow gripper to move left or right if forces are high
		if (fabs(dx) < 2)
			dx = 0;
		else
			dx = sign(dx) * (fabs(dx) - 2);
		if (fabs(dy) < 2)
			dy = 0;
		else
			dy = sign(dy) * (fabs(dy) - 2);

		//for z components make it easier to achieve greater velocities when forces are high
		if (dz>2)
		{
			dz=pow(dz-1,1.5)+2;
		}

		//limit z component
		if (dz > 10) /// (dz-1)*desired_velocity  maximal speed in counter direction
		{
			dz = 10;
		}

		//get final references
		ref.header.stamp = ros::Time::now();
		ref.twist.linear.x = desired_velocity * (dx); //* desired_vector[0] * (1 - dx);
		ref.twist.linear.y = desired_velocity * (dy); // * desired_vector[1] * (1 - dy);
		//z reference must have velocity 1 if forces are 0
		ref.twist.linear.z = desired_velocity * desired_vector[2] * (1 - dz);


		//if limiting forxces for 40 cycles send it on topic to start closing gripper
		static int force_limit_count = 0;
		if (fabs(forcez - force_z0) > 3) {
			force_limit_count++;
			if (force_limit_count > 40) {
				force_limiting = 1;
			}
		} else {
			force_limit_count = 0;
		}

		//if roll is too high, one side is on package and needs to start moving in positive direction to envelop the package
		if (roll > 0.05) {
			ref.twist.linear.y += 0.01;
//			ref.twist.linear.z-=0.01;

		}
		//if roll is too low, other side is on package and needs to start moving in negative direction to envelop the package
		if (roll < -0.05) {
			ref.twist.linear.y += -0.01;
//			ref.twist.linear.z-=0.01;

		}

		//limit maximal velocity
		if (fabs(ref.twist.linear.x) > desired_velocity * 1.01 * 5)
			ref.twist.linear.x = desired_velocity * 5;
		if (fabs(ref.twist.linear.y) > desired_velocity * 1.01 * 5)
			ref.twist.linear.y = desired_velocity * 5;
		if (fabs(ref.twist.linear.z) > desired_velocity * 1.01 * 9)
			ref.twist.linear.z = desired_velocity * 9;

		printf("final x y z %.2f %.2f %.2f\n", ref.twist.linear.x,
				ref.twist.linear.y, ref.twist.linear.z);

		//filter torque measurements
		tx = tx * 0.9 + 0.1 * force_torque.wrench.torque.x;
		ty = ty * 0.9 + 0.1 * force_torque.wrench.torque.y;
		tz = tz * 0.9 + 0.1 * force_torque.wrench.torque.z;

		//if smaller than 0.2, do not react
		if (fabs(tx) < 0.2) {
			tx = 0;
		} else {
			force_limiting = 1;
		}

		if (fabs(ty) < 0.2) {
			ty = 0;
		} else {
			force_limiting = 1;
		}
		if (fabs(force_torque.wrench.torque.z) < 0.4) {
			tz = 0;
		} else {
			force_limiting = 1;
		}
		std::cout << "force limiting " << force_limiting << " error"
				<< force_error[2] << "forcez forcez0 " << forcez << " "
				<< force_z0 << "tx ty " << tx << " " << ty << std::endl;

		double torque_gain = 0.02;
		//caluclate angular reference in a way to reduce the torque
		ref.twist.angular.x = -tx * torque_gain;
		ref.twist.angular.y = -ty * torque_gain;
		ref.twist.angular.z = -tz * torque_gain * 0;

		printf("force %5.2f %5.2f %5.2f\n", forcex, forcey, forcez);
		printf(
				"dx dy dz %.2f %.2f %.2f   %.2f  forceerror[2]  %.2f   %.2f %.2f %.2f\n",
				dx, dy, dz, forcez - force_z0, force_error[2], tx, ty, tz);
		//std::cout<<dx <<" "<<dy<<" "<<dz<<" "<<tx <<" "<<ty<< " "<<tz<<<<std::endl;
		//limit angular velocity
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

//limits acceleration on individual joints
void limit_acceleration_for_joint_servoing()
{
	printf("1\n");
	if (first == false) {
		//remember reference from previous step in variable _old
		servo_joint_reference_old = servo_joint_reference;
		time_joint_reference_old = time_joint_reference;
	}
	printf("2\n");
	time_joint_reference=ros::Time::now();
	if (first == true) {
		printf("3\n");
		servo_joint_reference=servo_joint_reference_received;
		//for first step set reference to 0
		for (int i = 0; i < 6; i++) {
			servo_joint_reference.velocities[i] = 0;
		}
		servo_joint_reference_old = servo_joint_reference;
		time_joint_reference_old = time_joint_reference;

	} else {
		printf("4\n");

		//limit acceleration between previous and current step
		ros::Duration diff = time_joint_reference - time_joint_reference_old;
		double dt = diff.toSec();
		//if there is delay detween references it could cause large jump
		if (dt > 0.2) {
			dt = 0.002;
		}
		printf("5\n");

		//if time between reference is 0, set it to 1ms
		if (dt < 0.001) {
			dt = 0.001;
		}
		printf("6\n");

		for (int i = 0; i < 6; i++) {
			double new_value = servo_joint_reference_received.velocities[i];
			servo_joint_reference.velocities[i]=servo_joint_reference_received.velocities[i];
			printf("7\n");

			//dx is difference between current and new reference
			double dx = servo_joint_reference_received.velocities[i]
					- servo_joint_reference_old.velocities[i];
			printf("7\n");
			//if acceleration if greater than max allowed acc, limit its value based on previous reference and max acc
			if (fabs(dx / dt) > max_acceleration) {
				new_value = servo_joint_reference_old.velocities[i]
						+ sign(dx) * max_acceleration * dt;
			}
			printf("9\n");

			//check if new reference value is less than reference in this and previous steps
			if (fabs(new_value) < fabs(servo_joint_reference_received.velocities[i]) || fabs(new_value)<fabs(servo_joint_reference_old.velocities[i])) {
				servo_joint_reference.velocities[i] = new_value;
			}
		}
	}
	first = false;

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "servoing_force_ur");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	int refresh_rate;
	std::string joint_states_topic, force_torque_topic,
			twist_velocity_cmd_topic, joint_velocity_cmd_topic, tool_pose_topic, servo_force_limiting_topic, servo_reference_topic, servo_joint_reference_topic;
	;
	std::string dynamixel_service;
	double desired_force, desired_velocity;

	//topic for listening joint states from robot
	nh_ns.param("joint_states_topic", joint_states_topic,
			(std::string) "/joint_states");

	//topic for listening force and torque from robot flange
	nh_ns.param("force_torque_topic", force_torque_topic,
			(std::string) "/wrench");

	//topic for commanding velocities in cartesian frame
	nh_ns.param("twist_desired_command", twist_velocity_cmd_topic,
			(std::string) "/servo_server/delta_twist_cmds");

	//topic for commanding velocities in joint frame
	nh_ns.param("joint_desired_command", joint_velocity_cmd_topic,
			(std::string) "/servo_server/delta_joint_cmds");


	//topic for receiving servoing reference in cartesian space
	nh_ns.param("servo_reference_topic", servo_reference_topic,
			(std::string) "/servo_reference");

	//topic for receiving servoing reference in joint space
	nh_ns.param("servo_joint_reference_topic", servo_joint_reference_topic,
			(std::string) "/servo_joint_reference");

	//topic for receiving transformation of tool pose in robot base frame
	nh_ns.param("tool_pose_topic", tool_pose_topic, (std::string) "/tool_pose");

	//maximal acceleration for joint servoing
	nh_ns.param("servo_force_limiting_topic", servo_force_limiting_topic, (std::string) "/servo_force_limiting");


	//maximal acceleration for joint servoing
	nh_ns.param("max_acceleration", max_acceleration, 0.2);

	//force for downward servoing
	nh_ns.param("desired_force", desired_force, 5.0);

	//desired velocity for downward servoing
	nh_ns.param("desired_velocity", desired_velocity, 0.01);



	//subscriber for joint states
	ros::Subscriber joints_subscriber = nh.subscribe(joint_states_topic, 1000,
			joints_callback);

	//subscirer for topics receiving servoing reference in cartesian space
	ros::Subscriber servo_reference_subscriber = nh.subscribe(
			servo_reference_topic, 1000, servo_reference_callback);

	//subscriber for topics receiving servoing reference in joint space
	ros::Subscriber servo_joint_reference_subscriber = nh.subscribe(
			servo_joint_reference_topic, 1000, servo_joint_reference_callback);

	//subscrbier for receiving force and torque on the flange
	ros::Subscriber force_torque_subscriber = nh.subscribe(force_torque_topic,
			1000, force_torque_callback);

	//subscriber for receiving tool pose transformation in robot base
	ros::Subscriber tool_pose_subscriber = nh.subscribe(tool_pose_topic, 1000,
			tool_pose_callback);

	geometry_msgs::TwistStamped reference;
	//publisher of servoing commands in cartesian space
	ros::Publisher velocity_reference_publisher = nh.advertise
			< geometry_msgs::TwistStamped > (twist_velocity_cmd_topic, 1000);

	//publisher of servoing commands in joint space
	ros::Publisher velocity_joint_reference_publisher = nh.advertise
			< control_msgs::JointJog > (joint_velocity_cmd_topic, 1000);

	//publisher that publish if servoing downward hit an obstacle
	ros::Publisher force_limiting_publisher = nh.advertise < std_msgs::Int32
			> ( servo_force_limiting_topic, 1000);

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

	//control loop
	while (ros::ok()) {
		force_limiting = 0;
		//claculate reference in cartesian space
		reference = calculate_reference(desired_force, desired_velocity);
		reference.header.stamp = ros::Time::now();

		servo_joint_reference.header.stamp = ros::Time::now();
		std::cout << reference << " received reference 1 "
				<< received_servo_reference_cartesian_space
				<< " received reference 2 "
				<< received_servo_reference_joint_space << std::endl;
		std::cout << servo_joint_reference << std::endl;

		//send corresponding reference depending what command was last given
		if (received_servo_reference_cartesian_space == 1) {
			velocity_reference_publisher.publish(reference);
		} else if (received_servo_reference_joint_space == 1) {
			limit_acceleration_for_joint_servoing();
			velocity_joint_reference_publisher.publish(servo_joint_reference);
		}

		//publish if force is limited (reached obstacle while servoing downward)
		force_limiting_msg.data = force_limiting;
		force_limiting_publisher.publish(force_limiting_msg);
		ros::spinOnce();
		loop_rate.sleep();

	}
}

