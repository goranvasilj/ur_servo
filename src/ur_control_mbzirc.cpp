#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
//#include <ncurses.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <ur_kinematics/ur_kin.h>
#include <ur_msgs/SetIO.h>
#include <control_msgs/JointJog.h>
enum OPERATION_MODE {
	AUTO = 0, MANUAL = 1, STOP = 2, FOLLOW_MODE = 3, TEST = 4, DOCKING_MODE = 5, FOLLOW_MODE_2 = 6, PICKUP_AFTER_WINCH = 7
};
enum STATES {
	IDLE = 0,
	GOTO_HOME = 1,
	SCAN = 2,
	GOTO_ABOVE = 3,
	SERVO_DOWN = 4,
	CLOSE_TOOL = 5,
	SERVO_UP = 6,
	GOTO_DROP = 7,
	OPEN_TOOL = 8,
	FINISHED = 9,
	FOLLOW = 10,
	GOTO_HOME_FOLLOW = 11,
	TEST_SERVO = 12,
	GOTO_HOME_DOCKING = 13,
	DOCKING = 14,
	GOTO_HOME_FOLLOW2 = 15,
	GOTO_HOME_FOR_PICKUP_AFTER_WINCH=16
};
bool uav_yaw_angle_received=false;
double uav_yaw_angle=0;
double uav_pickup_x=0;
double uav_pickup_y=0;
double uav_pickup_z=0;

double laser_pose[4][4];
double above_point[4][4];
double xref = 0, yref = 0, zref = 0, angle_final = 0;
double speed_linear = 0.02, speed_angular = 0.03, speed_joint_tracking = 0.06
		* 2 * 2, speed_joint = 0.06, joint_position_gain = 1.5;
double jointInfo[6];
OPERATION_MODE operation_mode = OPERATION_MODE::MANUAL;
STATES manual_command = STATES::IDLE;
STATES current_state = STATES::IDLE;
int force_limiting = 0;
int joints_received = 0;
ros::ServiceClient gripper_service;
ros::Publisher servo_reference_publisher;
ros::Publisher task_finished_publisher;
ros::Publisher tool_pose_publisher;
ros::Publisher servo_reference_joint_publisher;
ros::Time received_follow_pose_time = ros::Time(0);
geometry_msgs::PoseStamped follow_pose;
std::string joint_states_topic, force_torque_topic, velocity_cmd_topic,
		operation_mode_topic, manual_command_topic, joint_velocity_cmd_topic,
		object_location_update_topic, servo_force_limiting_topic,
		box_position_array_topic, follow_pose_topic, tool_pose_topic,
		gripper_service_name, base_link_enu_aligned_frame, lidar_sensor_frame;
//tf::TransformListener listener;

bool change_state = true;
int count_boxes = 0;
std::vector<double> x_box;
std::vector<double> y_box;
std::vector<double> z_box;
std::vector<double> angle_box;

//function which return signum of the input value
double sign(double x) {
	if (x < 0)
		return -1;
	if (x == 0)
		return 0;
	return 1;
}

//Callback for receiving box position from the camera
void box_position_array_callback(
		const std_msgs::Float32MultiArray::ConstPtr &msg) {
	if (current_state == STATES::SCAN) {
		if (count_boxes == 0) {
			x_box.clear();
			y_box.clear();
			z_box.clear();
			angle_box.clear();
		}
		x_box.push_back(msg->data[0]);
		y_box.push_back(msg->data[1]);
		z_box.push_back(msg->data[2]);
		angle_box.push_back(msg->data[5]);

		count_boxes++;
	} else {
		count_boxes = 0;
	}
}

//callback for receiving positions of individual joints
void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
	sensor_msgs::JointState js = *msg;
	joints_received = 1;
	for (int i = 0; i < 6; i++) {
		if (js.name[i] == "shoulder_pan_joint") {
			jointInfo[0] = js.position[i];
		}
		if (js.name[i] == "shoulder_lift_joint") {
			jointInfo[1] = js.position[i];
		}
		if (js.name[i] == "elbow_joint") {
			jointInfo[2] = js.position[i];
		}
		if (js.name[i] == "wrist_1_joint") {
			jointInfo[3] = js.position[i];
		}
		if (js.name[i] == "wrist_2_joint") {
			jointInfo[4] = js.position[i];
		}
		if (js.name[i] == "wrist_3_joint") {
			jointInfo[5] = js.position[i];
		}

	}
}

//Callback for receiving pose of the uav in base coordinate frame
void follow_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
	follow_pose = *msg;
	received_follow_pose_time = ros::Time::now();
}

//callback for receiving yaw angle from UAV when winch operation is done

void uav_yaw_angle_callback(const std_msgs::Float32::ConstPtr &msg)
{
	uav_yaw_angle_received=true;
	uav_yaw_angle=(*msg).data;
	double uav_pickup_x=follow_pose.pose.position.x;
	double uav_pickup_y=follow_pose.pose.position.y;
	double uav_pickup_z=follow_pose.pose.position.z;

}

//callback for setting operation mode
void operation_mode_callback(const std_msgs::Int32::ConstPtr &msg) {
	operation_mode = (OPERATION_MODE) (*msg).data;
	current_state = STATES::IDLE;
	if (operation_mode == OPERATION_MODE::TEST) {
		current_state = STATES::OPEN_TOOL;//STATES::TEST_SERVO;
		operation_mode = OPERATION_MODE::AUTO;
	}
	if (operation_mode == OPERATION_MODE::DOCKING_MODE) {
		current_state = STATES::GOTO_HOME_DOCKING;
		operation_mode = OPERATION_MODE::AUTO;
	}
	if (operation_mode== OPERATION_MODE::FOLLOW_MODE){
		current_state=STATES::FOLLOW;
		operation_mode = OPERATION_MODE::AUTO;	
	}
	if (operation_mode== OPERATION_MODE::FOLLOW_MODE_2){
		current_state=STATES::GOTO_HOME_FOLLOW2;
		operation_mode = OPERATION_MODE::AUTO;
	}
	if (operation_mode== OPERATION_MODE::PICKUP_AFTER_WINCH){
		current_state=STATES::GOTO_HOME_FOR_PICKUP_AFTER_WINCH;
		operation_mode = OPERATION_MODE::AUTO;
	}
	change_state = true;

}

//callback for linear servoing commands from keyborad
void keyboard_callback(const geometry_msgs::Twist::ConstPtr &msg) {
	xref = (*msg).linear.x * 0.01;
	yref = (*msg).linear.y * 0.01;
	zref = (*msg).linear.z * 0.01;

}

//callback for issuick manual commands
void manual_command_callback(const std_msgs::Int32::ConstPtr &msg) {
	manual_command = (STATES) (*msg).data;
}

//callback for receiving information if servoing is limiting force
void force_limiting_callback(const std_msgs::Int32::ConstPtr &msg) {

	force_limiting = (*msg).data;
}

//Transformation from kinematics frame of reference to pendant frame of reference
void prepare_transformation(double *T) {
	tf::Matrix3x3 rot1(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot2(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot3(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot4(-1, 0, 0, 0, -1, 0, 0, 0, 1);
	tf::Matrix3x3 rot5(0, -1, 0, 0, 0, -1, 1, 0, 0);

	rot2.setEulerZYX(0, 0, 3.14159265 / 2);
	rot3.setEulerZYX(0, 3.14159265 / 2, 0);

	rot4 = rot4.inverse();
	rot1 = rot4 * rot1 * rot5.inverse();
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			T[i * 4 + j] = rot1[i][j];
		}
	}

	double v1, v2, v3;
	v1 = T[0 * 4 + 3];
	v2 = T[1 * 4 + 3];
	v3 = T[2 * 4 + 3];
	T[0 * 4 + 3] = rot4[0][0] * v1 + rot4[0][1] * v2 + rot4[0][2] * v3;
	T[1 * 4 + 3] = rot4[1][0] * v1 + rot4[1][1] * v2 + rot4[1][2] * v3;
	T[2 * 4 + 3] = rot4[2][0] * v1 + rot4[2][1] * v2 + rot4[2][2] * v3;

}

//Inverse transformation from kinematics frame of reference to pendant frame of reference
void prepare_transformation_inverse(double *T) {
	tf::Matrix3x3 rot1(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot2(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot3(T[0 * 4 + 0], T[0 * 4 + 1], T[0 * 4 + 2], T[1 * 4 + 0],
			T[1 * 4 + 1], T[1 * 4 + 2], T[2 * 4 + 0], T[2 * 4 + 1],
			T[2 * 4 + 2]);
	tf::Matrix3x3 rot4(-1, 0, 0, 0, -1, 0, 0, 0, 1);
	tf::Matrix3x3 rot5(0, -1, 0, 0, 0, -1, 1, 0, 0);

	rot1 = rot4 * rot1 * rot5;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			T[i * 4 + j] = rot1[i][j];
		}
	}

	double v1, v2, v3;
	v1 = T[0 * 4 + 3];
	v2 = T[1 * 4 + 3];
	v3 = T[2 * 4 + 3];
	T[0 * 4 + 3] = rot4[0][0] * v1 + rot4[0][1] * v2 + rot4[0][2] * v3;
	T[1 * 4 + 3] = rot4[1][0] * v1 + rot4[1][1] * v2 + rot4[1][2] * v3;
	T[2 * 4 + 3] = rot4[2][0] * v1 + rot4[2][1] * v2 + rot4[2][2] * v3;

}

//servoing in joint space to reach defined joint configuration
bool go_to_joint(double *joints, double speed_joint, bool sync_arrival_of_all_joints) {

	control_msgs::JointJog ref;

	ref.joint_names.push_back("shoulder_pan_joint");
	ref.joint_names.push_back("shoulder_lift_joint");
	ref.joint_names.push_back("elbow_joint");
	ref.joint_names.push_back("wrist_1_joint");
	ref.joint_names.push_back("wrist_2_joint");
	ref.joint_names.push_back("wrist_3_joint");
	double joint_position_gain = 1.5;
	double sum = 0;
	double max_offset=fabs(joints[0] - jointInfo[0]);
	for (int i = 0; i < 6; i++) {
		double diff = joints[i] - jointInfo[i];
		sum = sum + fabs(diff);
		if (sync_arrival_of_all_joints == false)
		{
			double vel = diff * joint_position_gain;
			if (fabs(vel) > speed_joint) {
				vel = sign(vel) * speed_joint;
			}
			ref.velocities.push_back(vel);

			std::cout << vel << std::endl;
		}
		else
		{
			if (fabs(diff)>max_offset)
			{
				max_offset=fabs(diff);
			}
		}

	}
	if (sync_arrival_of_all_joints == true){
		for (int i = 0; i < 6; i++) {
			double diff = joints[i] - jointInfo[i];
			double scaling = fabs(diff)/max_offset;
			double vel = diff * joint_position_gain;

			if (fabs(vel) > speed_joint) {
				vel = sign(vel) * speed_joint*scaling;
			}
			ref.velocities.push_back(vel);
			std::cout << vel << std::endl;
		}
	}

	ref.header.stamp = ros::Time::now();
	servo_reference_joint_publisher.publish(ref);
	if (sum < 0.01) {
		return true;
	}
	return false;

}

//Servoing in cartesian space to reach defined pose
bool go_to(double *T_goal) {
	geometry_msgs::Twist ref;
	double T_start[16];
	//Calculate current pose from joints
	ur_kinematics::forward(jointInfo, T_start);

	prepare_transformation(T_start);
	tf::Matrix3x3 rot_start(T_start[0 * 4 + 0], T_start[0 * 4 + 1],
			T_start[0 * 4 + 2], T_start[1 * 4 + 0], T_start[1 * 4 + 1],
			T_start[1 * 4 + 2], T_start[2 * 4 + 0], T_start[2 * 4 + 1],
			T_start[2 * 4 + 2]);
	tf::Matrix3x3 rot_goal(T_goal[0 * 4 + 0], T_goal[0 * 4 + 1],
			T_goal[0 * 4 + 2], T_goal[1 * 4 + 0], T_goal[1 * 4 + 1],
			T_goal[1 * 4 + 2], T_goal[2 * 4 + 0], T_goal[2 * 4 + 1],
			T_goal[2 * 4 + 2]);

	//calculate rotation difference
	tf::Quaternion q_start, q_goal;
	rot_start.getRotation(q_start);
	rot_goal.getRotation(q_goal);

	tf::Matrix3x3 rot_diff(rot_start.inverse() * rot_goal);

	double yaw, pitch, roll;

	//convert quaternion to roll pitch, yaw
	rot_diff.getEulerYPR(yaw, pitch, roll);

	//calculate translation difference
	double dx, dy, dz, dist_lin, dist_ang;
	dx = T_goal[0 * 4 + 3] - T_start[0 * 4 + 3];
	dy = T_goal[1 * 4 + 3] - T_start[1 * 4 + 3];
	dz = T_goal[2 * 4 + 3] - T_start[2 * 4 + 3];

	double dx1, dy1, dz1;

	double current_roll, current_pitch, current_yaw;
	rot_start.getEulerYPR(current_yaw, current_pitch, current_roll);
	printf("current pitch roll yaw %.2f %.2f %.2f %.2f\n ", current_pitch,
			current_roll, current_yaw);

	printf("dx dy dz %.2f %.2f %.2f\n", dx, dy, dz);
	printf("dpitch droll dyaw %.2f %.2f %.2f\n", pitch, roll, yaw);

	//scale down linear velocity if close to final point
	dist_lin = sqrt(dx * dx + dy * dy + dz * dz);
	double scaling = 1;
	if (dist_lin < 0.1)
		scaling = dist_lin / 0.1;
	if (dist_lin < 0.001)
		dist_lin = 0.001;

	dist_ang = sqrt(yaw * yaw + pitch * pitch + roll * roll);

	//calculate individiual linear velocities
	double xref1, yref1, zref1;
	xref1 = -dx / dist_lin * speed_linear * scaling;
	yref1 = -dy / dist_lin * speed_linear * scaling;
	zref1 = dz / dist_lin * speed_linear * scaling;

	ref.linear.x = T_start[0 * 4 + 0] * xref1 + T_start[1 * 4 + 0] * yref1
			+ T_start[2 * 4 + 0] * zref1;
	ref.linear.y = T_start[0 * 4 + 1] * xref1 + T_start[1 * 4 + 1] * yref1
			+ T_start[2 * 4 + 1] * zref1;
	ref.linear.z = T_start[0 * 4 + 2] * xref1 + T_start[1 * 4 + 2] * yref1
			+ T_start[2 * 4 + 2] * zref1;
	if (fabs(ref.linear.x) > speed_linear)
		ref.linear.x = 0;
	if (fabs(ref.linear.y) > speed_linear)
		ref.linear.y = 0;
	if (fabs(ref.linear.z) > speed_linear)
		ref.linear.z = 0;

	//scale down angular velocity if close to final orientation
	double scaling1 = 1;
	if (dist_ang < 0.1)
		scaling1 = dist_ang / 0.1;
	if (dist_ang < 0.001)
		dist_ang = 0.001;

	//calculate individual angular velocity
	ref.angular.x = yaw / dist_ang * speed_angular * scaling1;
	ref.angular.y = -pitch / dist_ang * speed_angular / 2 * scaling1;
	ref.angular.z = roll / dist_ang * speed_angular / 2 * scaling1;

	if (fabs(ref.angular.x) > speed_angular)
		ref.angular.x = 0;
	if (fabs(ref.angular.y) > speed_angular)
		ref.angular.y = 0;
	if (fabs(ref.angular.z) > speed_angular)
		ref.angular.z = 0;

	std::cout << ref << std::endl;
	//publish servo command
	servo_reference_publisher.publish(ref);

	//return true if in position
	if (dist_lin < 0.01 && dist_ang < 0.05)
		return true;
	else
		return false;

}

//go to home position for grabbing object
bool goto_home() {
	//go to predefined home position
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] = -90. / 180 * 3.14159265;
	joint[1] = -90. / 180 * 3.14159265;
	joint[2] = -40. / 180 * 3.14159265;
	joint[3] = -90. / 180 * 3.14159265;
	joint[4] = 80. / 180 * 3.14159265;
	joint[5] = -90. / 180 * 3.14159265;

	double T[4][4];
	ur_kinematics::forward(joint, &T[0][0]);
	prepare_transformation(&T[0][0]);

/*	printf("home\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T[i][j]);
		}
		printf("\n");

	}*/
	for (int i=0;i<6;i++)
	{
		printf("%.2f %.2f\n",joint[i],jointInfo[i]);
	}
	printf("\n");
	change_state = false;
	return go_to_joint(joint, speed_joint,false);
}

//go to the home position for tracking UAV
bool goto_home_follow() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] =-3.2;// -90. / 180 * 3.14159265;
	joint[1] = -65. / 180 * 3.14159265;
	joint[2] = -40. / 180 * 3.14159265;
	joint[3] = -105. / 180 * 3.14159265;
	joint[4] = 80. / 180 * 3.14159265;
	joint[5] = -90. / 180 * 3.14159265;

	double T[4][4];
	ur_kinematics::forward(joint, &T[0][0]);
	prepare_transformation(&T[0][0]);

	printf("home\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T[i][j]);
		}
		printf("\n");

	}
	change_state = false;
	return go_to_joint(joint, speed_joint, false);
}


//go to the home position for tracking UAV 2
bool goto_home_follow2() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] = -90. / 180 * 3.14159265;
	joint[1] = -90. / 180 * 3.14159265;
	joint[2] = -60. / 180 * 3.14159265;
	joint[3] = -50. / 180 * 3.14159265;
	joint[4] = 80. / 180 * 3.14159265;
	joint[5] = -90. / 180 * 3.14159265;

	double T[4][4];
	ur_kinematics::forward(joint, &T[0][0]);
	prepare_transformation(&T[0][0]);

	printf("home\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T[i][j]);
		}
		printf("\n");

	}
	change_state = false;
	return go_to_joint(joint, speed_joint, false);
}
//go to the home position for docking procedure
bool goto_home_docking() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] = -90. / 180 * 3.14159265;
	joint[1] = -101. / 180 * 3.14159265;
	joint[2] = -16. / 180 * 3.14159265;
	joint[3] = -92. / 180 * 3.14159265;
	joint[4] = 92. / 180 * 3.14159265;
	joint[5] = -90. / 180 * 3.14159265;

	double T[4][4];
	ur_kinematics::forward(joint, &T[0][0]);
	prepare_transformation(&T[0][0]);

	printf("home\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T[i][j]);
		}
		printf("\n");

	}
	change_state = false;
	return go_to_joint(joint, speed_joint, false);
}
//go to the calculated point above the object to be grabbed
bool goto_above() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	double above_point1[4][4];
	/*
	 //manually set above point
	 joint[0] = -84.88 / 180 * 3.14159265;
	 joint[1] = -129.86 / 180 * 3.14159265;
	 joint[2] = -53.65 / 180 * 3.14159265;
	 joint[3] = -85.92 / 180 * 3.14159265;
	 joint[4] = 92.73 / 180 * 3.14159265;
	 joint[5] = -309.80 / 180 * 3.14159265;

	 double T[4][4];
	 ur_kinematics::forward(joint, &T[0][0]);
	 prepare_transformation(&T[0][0]);
	 printf("above point\n");
	 for (int i = 0; i < 4; i++) {
	 for (int j = 0; j < 4; j++) {
	 printf("%.4f ", T[i][j]);
	 //above_point[i][j]=T[i][j];
	 }
	 printf("\n");
	 }*/
	/*above_point[0][0]=-1; above_point[0][1]=0; above_point[0][2]=0;above_point[0][3]=1;
	 above_point[1][0]=0; above_point[1][1]=1; above_point[1][2]=0;above_point[1][3]=1;
	 above_point[2][0]=0;above_point[2][1]=0; above_point[2][2]=-1;above_point[2][3]=1;
	 above_point[3][0]=0; above_point[3][1]=0; above_point[3][2]=0; above_point[3][3]=1;
	 */

	//above point calculated from camera information
	printf("\n above angle %.4f\n", angle_final);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.4f ", above_point[i][j]);
			above_point1[i][j] = above_point[i][j];
		}
		printf("\n");
	}

	//find all solutions for inverse kinematics to reach above point
	double sols[8 * 6];
	prepare_transformation_inverse(&above_point1[0][0]);
	int k = ur_kinematics::inverse(&above_point1[0][0], sols, 0);
	int found = 0;

	//select the right solution
	for (int i = 0; i < k; i++) {
		for (int j = 0; j < 6; j++) {
			if (sols[i * 6 + j] - jointInfo[j] > 3.14159265) {

				sols[i * 6 + j] -= 2 * 3.14159265;
			}
			if (-sols[i * 6 + j] + jointInfo[j] > 3.14159265) {

				sols[i * 6 + j] += 2 * 3.14159265;
			}
			if (j == 0) {
				if (sols[i * 6 + j] > 0) {
					sols[i * 6 + j] -= 2 * 3.14159265;
				}
			}
			if (j == 5) {
				if (sols[i * 6 + j] < -3.14159) {
					sols[i * 6 + j] += 3.14159265;
				}
			}
			printf("%.3f ", sols[i * 6 + j]);
		}
		printf("\n");
		//printf("%.2f %.2f %.2f\n",sols[i*6+0],-150/3.14159*180,-30/3.14159);
		//printf("%.2f %.2f %.2f\n",sols[i*6+1],-170/3.14159*180,-30/3.14159);
		//printf("%.2f %.2f %.2f\n",sols[i*6+2],-150/3.14159*180,-30/3.14159);
//		joint[0] = 180 / 180 * 3.14159265;
//		joint[1] = -90. / 180 * 3.14159265;
//		joint[2] = -60. / 180 * 3.14159265;
//		joint[3] = -66. / 180 * 3.14159265;
//		joint[4] = 80. / 180 * 3.14159265;
//		joint[5] = -90. / 180 * 3.14159265;
		if (sols[i * 6 + 0] > -135 * 3.14159 / 180
				&& sols[i * 6 + 0] < -45 * 3.14159 / 180) {
			if (sols[i * 6 + 1] > -170 * 3.14159 / 180
					&& sols[i * 6 + 1] < -90 * 3.14159 / 180) {
				if (sols[i * 6 + 2] > -120 * 3.14159 / 180
						&& sols[i * 6 + 2] < -30 * 3.14159 / 180) {
					if (sols[i * 6 + 4] > 0 * 3.14159 / 180
							&& sols[i * 6 + 4] < 120 * 3.14159 / 180) {
						printf("good ");
						for (int j = 0; j < 6; j++) {
							printf("%.3f ", sols[i * 6 + j]);
							joint[j] = sols[i * 6 + j];
						}
						found = 1;
						printf("\n");
					}
				}
			}
		}
	}
	static bool first_fail=false;
	if (found == 0) {

		printf("no good solutions\n");
		if (first_fail==false)
		{
			first_fail=true;
			std_msgs::Bool msg;
			msg.data=false;
			task_finished_publisher.publish(msg);
		}
		return false;
	}

	//change joint 5 to right range, since the tool is symetrical
	if (joint[5] - jointInfo[5] > 3.14159 / 2) {
		joint[5] = joint[5] - 3.14159265;
	}
	if (-joint[5] + jointInfo[5] > 3.14159 / 2) {
		joint[5] = joint[5] + 3.14159265;
	}
	if (joint[5] > 6.2) {
		joint[5] = joint[5] - 3.14159265;
	}
	if (joint[5] < -6.2) {
		joint[5] = joint[5] + 3.14159265;
	}

	printf("\n");
	printf("\n");
	change_state = false;
	//return false;

	return go_to_joint(joint, speed_joint, true);
	//return go_to(&above_point[0][0]);

}

//Call to close gripper on the tool
bool close_tool() {
	static ros::Time start = ros::Time::now();
	if (change_state) {
		start = ros::Time::now();
		ur_msgs::SetIO srv;
		srv.request.fun = 1;
		srv.request.pin = 17;
		srv.request.state = 1.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 1" << std::endl;
		}

		srv.request.fun = 1;
		srv.request.pin = 16;
		srv.request.state = 0.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 2" << std::endl;
		}
	}
	change_state = false;
	ros::Duration diff = ros::Time::now() - start;
	if (diff.toSec() > 45) {
		return true;
	} else {
		return false;
	}
}

//Call to open gripper on tool
bool open_tool() {
	static ros::Time start = ros::Time::now();
	if (change_state) {
		start = ros::Time::now();
		ur_msgs::SetIO srv;
		srv.request.fun = 1;
		srv.request.pin = 17;
		srv.request.state = 0.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 1" << std::endl;
		}

		srv.request.fun = 1;
		srv.request.pin = 16;
		srv.request.state = 1.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 2" << std::endl;
		}
	}
	change_state = false;
	ros::Duration diff = ros::Time::now() - start;
	if (diff.toSec() > 45) {
		return true;
	} else {
		return false;
	}
}

//Call to stop gripper movement
bool stop_tool() {
	if (change_state) {
		ur_msgs::SetIO srv;
		srv.request.fun = 1;
		srv.request.pin = 17;
		srv.request.state = 0.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 1" << std::endl;
		}

		srv.request.fun = 1;
		srv.request.pin = 16;
		srv.request.state = 0.0;
		if (gripper_service.call(srv)) {
			std::cout << "called_service 2" << std::endl;
		}
	}
	change_state = false;
	return true;
}

//stop joint movement
bool stop_movement() {
	geometry_msgs::Twist ref;
	ref.linear.x = 0;
	ref.linear.y = 0;
	ref.linear.z = 0;
	ref.angular.x = 0;
	ref.angular.y = 0;
	ref.angular.z = 0;
	servo_reference_publisher.publish(ref);
	return true;
}

//lawnmower patterning scanning for the object
bool scan() {
	static double T_start[16] = { 0 };
	static double joints_start[6] = { 0 };
	double scan_limit = 0.5, increment = -0.15;
	static int scan_line = 0;
	static int increment_line = 0;
	static ros::Time start = ros::Time::now();
	static tf::Quaternion q_start, q_goal;
	static double pitch0 = 0;
	geometry_msgs::Twist ref;
	if (change_state) {
		//first call stop and wait for 2 seconds
		for (int i = 0; i < 6; i++) {
			joints_start[i] = jointInfo[i];
		}
		start = ros::Time::now();
		ref.linear.x = 0;
		ref.linear.y = 0;
		ref.linear.z = 0;
		ref.angular.x = 0;
		ref.angular.y = 0;
		ref.angular.z = 0;
		scan_line = 0;
		increment = 0;
		servo_reference_publisher.publish(ref);
		pitch0 = 0;

	}
	//wait two seconds after going into this state
	ros::Duration diff = ros::Time::now() - start;
	if (diff.toSec() > 2) {
		double joints[6] = { 0 };
		for (int i = 0; i < 6; i++) {
			joints[i] = joints_start[i];
		}
		//move from -0.3 to +0.3
		if (scan_line % 2 == 0) {
			joints[0] = joints[0] + 0.7;

		} else {
			joints[0] = joints[0] - 0.4;
		}
		//increase pictch of the viewangle
		joints[3] = joints[3] + pitch0;

		if (go_to_joint(joints, speed_joint, false)) {
			scan_line = scan_line + 1;
			pitch0 += increment;
		}

	}
	//go to 0.6 from home position
	if (fabs(pitch0) > 0.62) {
		ref.linear.x = 0;
		ref.linear.y = 0;
		ref.linear.z = 0;
		ref.angular.x = 0;
		ref.angular.y = 0;
		ref.angular.z = 0;
		scan_line = 0;
		increment = 0;
		servo_reference_publisher.publish(ref);
		return true;
	}
	change_state = false;
	return false;
}

//manually servo roll and pitch
bool servo_roll_pitch(double angle1, double angle2) {
	geometry_msgs::Twist ref;
	ref.linear.x = 0;
	ref.linear.y = 0;
	ref.linear.z = 0;
	if (fabs(angle1) > 0.1) {
		angle1 = 0.01 * sign(angle1);
	} else {
		angle1 = angle1 * 0.1;
	}
	if (fabs(angle2) > 0.1) {
		angle2 = 0.01 * sign(angle2);
	} else {
		angle1 = angle1 * 0.1;
	}

	ref.angular.x = angle1;
	ref.angular.y = angle2;
	ref.angular.z = 0;
	servo_reference_publisher.publish(ref);
	change_state = false;
	if (force_limiting == 1) {
		return true;
	} else {
		return false;
	}
}

//follow UAV with robot arm
bool follow() {
	static tf::Quaternion q_start, q_goal;
	geometry_msgs::Twist ref;
	ros::Duration d = ros::Time::now() - received_follow_pose_time;
	static double joints_start[6];
	if (change_state) {
		//remember starting joint position
		for (int i = 0; i < 6; i++) {
			joints_start[i] = jointInfo[i];
		}
	}
	//if there is no update in position for 2 seconds stop servoing
	if (d.toSec() > 2) {
		ref.linear.x = 0;
		ref.linear.y = 0;
		ref.linear.z = 0;
		ref.angular.x = 0;
		ref.angular.y = 0;
		ref.angular.z = 0;

		servo_reference_publisher.publish(ref);
	} else {
		double joints[6] = { 0 };
		double x = follow_pose.pose.position.x;
		double y = follow_pose.pose.position.y;
		double z = follow_pose.pose.position.z;
		//convert global uav position to tool frame
		double x_a = x * laser_pose[0][0] + y * laser_pose[0][1]
				+ z * laser_pose[0][2] + laser_pose[0][3];
		double y_a = x * laser_pose[1][0] + y * laser_pose[1][1]
				+ z * laser_pose[1][2] + laser_pose[1][3];
		double z_a = x * laser_pose[2][0] + y * laser_pose[2][1]
				+ z * laser_pose[2][2] + laser_pose[2][3];

		//calculate difference from center in x and y direction
		double angle1 = atan(y_a / x_a);
		double angle2 = atan(z_a / x_a);
		printf(
				"angle 1 angle2 %.4f %.4f  x y z %.3f %.3f %.3f  xa ya za %.3f %.3f %.3f\n",
				angle1, angle2, x, y, z, x_a, y_a, z_a);
		for (int i = 0; i < 6; i++) {
			joints[i] = joints_start[i];
		}
		//base rotation solves x offset
		joints[0] = jointInfo[0] + angle1;
		joints[3] = jointInfo[3];
		joints[4] = jointInfo[4];
		//wrist 1 solves y offset
		joints[3] = jointInfo[3] + angle2;
		if (go_to_joint(joints, speed_joint_tracking, false)) {

		}
	}
	change_state = false;
	return false;
}

//go top predefined drop position
bool goto_drop() {
	stop_movement();
	return true;
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] = 67.24 / 180 * 3.14159265;
	joint[1] = -130.01 / 180 * 3.14159265;
	joint[2] = -32.38 / 180 * 3.14159265;
	joint[3] = -105.51 / 180 * 3.14159265;
	joint[4] = 89.92 / 180 * 3.14159265;
	joint[5] = -168.52 / 180 * 3.14159265;

	double T[4][4];
	ur_kinematics::forward(joint, &T[0][0]);
	prepare_transformation(&T[0][0]);
	printf("above point\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.4f ", T[i][j]);
		}
		printf("\n");
	}
	return go_to_joint(&T[0][0], speed_joint, false);
}

//servo with defined force down toward object
bool servo_down() {
	geometry_msgs::Twist ref;
	ref.linear.x = 0;
	ref.linear.y = 0;
	ref.linear.z = 0.01;
	ref.angular.x = 0;
	ref.angular.y = 0;
	ref.angular.z = 0;
	servo_reference_publisher.publish(ref);
	change_state = false;
	if (force_limiting == 1) {
		return true;
	} else {
		return false;
	}
}

//servo up with picked up object
bool servo_up() {
	static double initial_z = 0;

	double T_current[16];
	ur_kinematics::forward(jointInfo, T_current);
	prepare_transformation(T_current);
	if (change_state) {
		initial_z = T_current[2 * 4 + 3];
	}
	geometry_msgs::Twist ref;
	ref.linear.x = 0;
	ref.linear.y = 0;
	ref.linear.z = -0.02;
	ref.angular.x = 0;
	ref.angular.y = 0;
	ref.angular.z = 0;
	std::cout << ref << std::endl;
	servo_reference_publisher.publish(ref);
	change_state = false;
	printf("%.2f\n", T_current[2 * 4 + 3] - initial_z);
	if (T_current[2 * 4 + 3] - initial_z > 0.2) {
		return true;
	} else {
		return false;
	}

}

//stop robot and tool movement
void stop() {
	if (change_state) {
		stop_tool();
	}
	stop_movement();
	change_state = false;
}

//caluclate above pose when pose is received from uav after winch final position
bool calculate_above_pose_from_uav() {

	//above position is 34 cm above object (14cm object height + 20cm)
	double x_final, y_final, z_final;
	x_final = uav_pickup_x;
	y_final = uav_pickup_y;
	z_final = uav_pickup_z -0.30;
	angle_final = 6.28 - uav_yaw_angle + 3.14159 / 2;

	//fill transformation matrix
	above_point[0][3] = x_final;
	above_point[1][3] = y_final;
	above_point[2][3] = z_final;
	above_point[3][3] = 1;
	above_point[0][0] = -sin(angle_final);
	above_point[0][1] = -cos(angle_final);
	above_point[0][2] = 0;
	above_point[1][0] = -cos(angle_final);
	above_point[1][1] = sin(angle_final);
	above_point[1][2] = 0;
	above_point[2][0] = 0;
	above_point[2][1] = 0;
	above_point[2][2] = -1;
	above_point[3][0] = 0;
	above_point[3][1] = 0;
	above_point[3][2] = 0;
	above_point[3][3] = 1;

}

//calculate robot pose directly above the object
bool calculate_above_pose() {
	if (count_boxes > 0) {
		double x_final, y_final, z_final;

		//get median of x,y,z and angle
		std::sort(x_box.begin(), x_box.end());
		std::sort(y_box.begin(), y_box.end());
		std::sort(z_box.begin(), z_box.end());
		std::sort(angle_box.begin(), angle_box.end());

		//above position is 34 cm above object (14cm object height + 20cm)
		x_final = x_box[count_boxes / 2];
		y_final = y_box[count_boxes / 2];
		z_final = z_box[count_boxes / 2] + 0.44;
		angle_final = 6.28 - angle_box[count_boxes / 2] + 3.14159 / 2;

		//fill transformation matrix
		above_point[0][3] = x_final;
		above_point[1][3] = y_final;
		above_point[2][3] = z_final;
		above_point[3][3] = 1;
		above_point[0][0] = -sin(angle_final);
		above_point[0][1] = -cos(angle_final);
		above_point[0][2] = 0;
		above_point[1][0] = -cos(angle_final);
		above_point[1][1] = sin(angle_final);
		above_point[1][2] = 0;
		above_point[2][0] = 0;
		above_point[2][1] = 0;
		above_point[2][2] = -1;
		above_point[3][0] = 0;
		above_point[3][1] = 0;
		above_point[3][2] = 0;
		above_point[3][3] = 1;

		return true;
	}
	return false;
}

//Update function for robot arm main state machine
void update() {
	if (operation_mode == OPERATION_MODE::STOP) {
		stop();

	} else {
		switch (current_state) {
		case STATES::IDLE:
			//waiting start of mission, either change state or go to auto mode
			stop();
			if (operation_mode == OPERATION_MODE::AUTO)
				current_state = GOTO_HOME;
			break;
		case STATES::GOTO_HOME:
			//go to home for grabbing
			if (goto_home()) {
				current_state = STATES::SCAN;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::SCAN:
			//lawnmower scanning for object
			if (scan()) {
				if (calculate_above_pose()) {
					current_state = STATES::GOTO_ABOVE;
					if (operation_mode == OPERATION_MODE::MANUAL)
						operation_mode = OPERATION_MODE::STOP;
					change_state = true;
				} else {
					std_msgs::Bool msg;
					msg.data=false;
					task_finished_publisher.publish(msg);
					current_state = STATES::GOTO_HOME;
					if (operation_mode == OPERATION_MODE::MANUAL)
						operation_mode = OPERATION_MODE::STOP;
					change_state = true;
				}
			}
			break;
		case STATES::GOTO_ABOVE:
			//go to above found object
			if (goto_above()) {
				current_state = STATES::SERVO_DOWN;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::SERVO_DOWN:
			//servo down to picku up object
			if (servo_down()) {
				current_state = STATES::CLOSE_TOOL;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::CLOSE_TOOL:
			// close  gripper
			if (close_tool()) {
				current_state = STATES::SERVO_UP;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::FINISHED:
			// stop all
			stop();
			break;
		case STATES::SERVO_UP:
			//servo up
			if (servo_up()) {
				current_state = STATES::GOTO_DROP;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::GOTO_DROP:
			// go to drop off position
			if (goto_drop()) {
				current_state = STATES::OPEN_TOOL;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::OPEN_TOOL:
			//open gripper
			if (open_tool()) {
				current_state = STATES::FINISHED;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				std_msgs::Bool msg;
				msg.data=true;
				task_finished_publisher.publish(msg);
				stop();
				change_state = true;
			}
			break;
		case STATES::GOTO_HOME_FOLLOW:
			//go to home position for uav tracking
			if (goto_home_follow()) {
				current_state = STATES::FOLLOW;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::GOTO_HOME_FOLLOW2:
			//go to home position for uav tracking
			if (goto_home_follow()) {
				current_state = STATES::FINISHED;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				stop();
			}
			break;
		case STATES::GOTO_HOME_FOR_PICKUP_AFTER_WINCH:
			//go to home position for pciking up object based on last UAV pose (pulled by the winch)
			if (goto_home()) {
				calculate_above_pose_from_uav();
				current_state = STATES::GOTO_ABOVE;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::TEST_SERVO:
			//testing servo_mode
			if (servo_down()) {

			}
			break;
		case STATES::FOLLOW:
			//track uav based on received position
			if (follow()) {
				current_state = STATES::FINISHED;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::GOTO_HOME_DOCKING:
			//go to home position for docking procedure
			if (goto_home_docking()) {
				current_state = STATES::FOLLOW;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::DOCKING:
			//docking procedure

			current_state = STATES::FINISHED;
			if (operation_mode == OPERATION_MODE::MANUAL)
				operation_mode = OPERATION_MODE::STOP;
			change_state = true;

			break;

		}
	}
}

//find transform form base_link frame to lidar frame for transformation of received uav pose in robot base frame to lidar frame
void read_transform() {
	static tf::TransformListener listener;
	tf::StampedTransform transform;
	try {
		listener.lookupTransform(lidar_sensor_frame,
				base_link_enu_aligned_frame, ros::Time(0), transform);
		//transform to transformation matrix
		tf::Matrix3x3 mat = transform.getBasis();
		tf::Vector3 vec = transform.getOrigin();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				laser_pose[i][j] = mat[i][j];
			}
		}
		laser_pose[3][0] = 0;
		laser_pose[3][1] = 0;
		laser_pose[3][2] = 0;
		laser_pose[3][3] = 1;
		laser_pose[0][3] = vec[0];
		laser_pose[1][3] = vec[1];
		laser_pose[2][3] = vec[2];
		printf("Laser pose\n");
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				printf("%.2f ", laser_pose[i][j]);
			}
			printf("\n");
		}
		printf("\n");
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
		ros::Duration(1.0).sleep();
	}

}

//intialize publishers, subscribers and services
void init(ros::NodeHandle nh_ns, ros::NodeHandle nh) {

	//topic for listening joint states from robot
	nh_ns.param("joint_states_topic", joint_states_topic,
			(std::string) "/joint_states");
	std::cout << joint_states_topic << std::endl;
	//topic for changing operation mode
	nh_ns.param("operation_mode", operation_mode_topic,
			(std::string) "/operation_mode");

	//topic for sending manual commands
	nh_ns.param("manual_command", manual_command_topic,
			(std::string) "/manual_command");

	//topic for receiving information if servoing is limiting force
	nh_ns.param("servo_force_limiting", servo_force_limiting_topic,
			(std::string) "/servo_force_limiting");

	//topic for receiving position of the object detected by the camera
	nh_ns.param("box_position_array_topic", box_position_array_topic,
			(std::string) "/box_position");

	//topic for receiving pose of UAV
	nh_ns.param("follow_pose_topic", follow_pose_topic,
			(std::string) "/hawk1/uav_pose");

	//topic to publish pose of tool in base coordinate system
	nh_ns.param("tool_pose_topic", tool_pose_topic, (std::string) "/tool_pose");

	//topic to send velocity commands in cartesian space
	nh_ns.param("velocity_cmd_topic", velocity_cmd_topic,
			(std::string) "/servo_reference");

	//topic to send velocity commands in joint space
	nh_ns.param("joint_velocity_cmd_topic", joint_velocity_cmd_topic,
			(std::string) "/servo_joint_reference");

	//topic to send velocity commands in cartesian space
	nh_ns.param("gripper_service_name", gripper_service_name,
			(std::string) "/ur_hardware_interface/set_io");

	//name of base_link_enu_aligned frame
	nh_ns.param("base_link_enu_aligned_frame", base_link_enu_aligned_frame,
			(std::string) "/base_link_enu_aligned");

	//name of lidar sensor frame
	nh_ns.param("lidar_sensor_frame", lidar_sensor_frame,
			(std::string) "/os_sensor");

	//joint servoing speed of individual joints
	nh_ns.param("joint_servoing_speed", speed_joint, 0.06);

	//target tracking speed of individual joints
	nh_ns.param("joint_tracking_speed", speed_joint_tracking, 0.06 * 2 * 2);

	//gain for joint position loop
	nh_ns.param("joint_position_loop_gain", joint_position_gain, 1.5);

	//service for setting i/o on robot - used for operating gripper
	gripper_service = nh.serviceClient < ur_msgs::SetIO
			> (gripper_service_name);

	//subcriber to joint states from robot controller
	static ros::Subscriber joints_subscriber = nh.subscribe(joint_states_topic,
			1, joints_callback);

	//subscriber for receiving pose of uav in robot base frame
	static ros::Subscriber follow_pose_subscriber = nh.subscribe(
			follow_pose_topic, 1, follow_pose_callback);

	//subcriber for receiving pose of large object from camera
	static ros::Subscriber box_position_subscriber = nh.subscribe(
			box_position_array_topic, 1, box_position_array_callback);

	//subscriber for seting operating mode
	static ros::Subscriber operating_mode_subscriber = nh.subscribe(
			operation_mode_topic, 1, operation_mode_callback);

	//subscriber for receiving of manual servoing commands
	static ros::Subscriber keyboard_subscriber = nh.subscribe("/cmd_vel", 1,
			keyboard_callback);

	//subscriber to information if servoing is limitting force, which is a signal to close the tool
	static ros::Subscriber servo_force_limiting_subscriber = nh.subscribe(
			servo_force_limiting_topic, 1, force_limiting_callback);

	//subscriber to receive manual commands if the system is in manual operating mode
	static ros::Subscriber manual_command_subscriber = nh.subscribe(
			manual_command_topic, 1, manual_command_callback);


	//subscriber to receive yaw angle when winch operation is done, and UAV is ready to fly away
	static ros::Subscriber uav_yaw_angle = nh.subscribe(
			"/ur_winch_done", 1, uav_yaw_angle_callback);

	//publisher for servoing commands in cartesian frema
	geometry_msgs::TwistStamped reference;
	servo_reference_publisher = nh.advertise < geometry_msgs::Twist
			> (velocity_cmd_topic, 1);

	//publisher for sending message when mission is done
	task_finished_publisher = nh.advertise < std_msgs::Bool
			> ("/big_object_finished", 1);

	//publisher for servoing commands in joint frame
	servo_reference_joint_publisher = nh.advertise < control_msgs::JointJog
			> (joint_velocity_cmd_topic, 1);

	//publisher for sending pose of tool for tracking LIDAR
	tool_pose_publisher = nh.advertise < std_msgs::Float32MultiArray
			> (tool_pose_topic, 1);

	//set initial state and mode of operation
//	operation_mode = OPERATION_MODE::FOLLOW_MODE;
//	current_state = STATES::GOTO_HOME_FOLLOW;
	operation_mode = OPERATION_MODE::STOP;
	current_state = STATES::IDLE;
	//	current_state=STATES::SCAN;
//	current_state=STATES::TEST_SERVO;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "ur_control_mbzirc");

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");

	//intialize publishers, subscribers and services
	init(nh_ns, nh);

	int refresh_rate;

	jointInfo[0] = 0;
	jointInfo[1] = 0;
	jointInfo[2] = 0;
	jointInfo[3] = 0;
	jointInfo[4] = 0;
	jointInfo[5] = 0;
	double T_start[16];
	ros::Rate loop_rate(20);

	//main loop

	while (ros::ok()) {
		//find transform form base_link frame to lidar frame for transformation of received uav pose in robot base frame to lidar frame
		if (operation_mode == OPERATION_MODE::AUTO) {
			if (current_state == STATES::GOTO_HOME_FOLLOW
					|| current_state == STATES::FOLLOW) {
				read_transform();
			}
		}

		//update only if actual joints location from robot are available
		if (joints_received == 1) {
			std::cout << "Mode " << current_state << "operation mode "
					<< operation_mode << std::endl;

			//update state machine
			update();
		}
		if (joints_received == 1) {
			// publish tool pose in base_link frame (array with 4x4 matrix)
			ur_kinematics::forward(jointInfo, T_start);
			prepare_transformation(T_start);

			std_msgs::Float32MultiArray msg;
			std::vector<float> vec;
			for (int i = 0; i < 4; i++)
				for (int j = 0; j < 4; j++)

					vec.push_back(T_start[i * 4 + j]);
			msg.data = vec;
			tool_pose_publisher.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
}

