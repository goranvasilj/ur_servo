#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
//#include <ncurses.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <stdio.h>
#include <ur_kinematics/ur_kin.h>
#include <ur_msgs/SetIO.h>
#include <control_msgs/JointJog.h>
enum OPERATION_MODE {
	AUTO = 0, MANUAL = 1, STOP = 2
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
	FINISHED = 9
};

double above_point[4][4];
double xref = 0, yref = 0, zref = 0, angle_final = 0;
double speed_linear = 0.02, speed_angular = 0.03, speed_joint=0.06;
double jointInfo[6];
OPERATION_MODE operation_mode = OPERATION_MODE::MANUAL;
STATES manual_command = STATES::IDLE;
STATES current_state = STATES::IDLE;
int object_location_update = 0;
int force_limiting = 0;
int joints_received = 0;
ros::ServiceClient gripper_service;
ros::Publisher servo_reference_publisher;
ros::Publisher servo_reference_joint_publisher;
bool change_state = true;
int count_boxes = 0;
std::vector<double> x_box;
std::vector<double> y_box;
std::vector<double> z_box;
std::vector<double> angle_box;
double sign(double x) {
	if (x < 0)
		return -1;
	if (x == 0)
		return 0;
	return 1;
}

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
void joints_callback(const sensor_msgs::JointState::ConstPtr &msg) {
	sensor_msgs::JointState js = *msg;
	joints_received = 1;
//	std::cout<<*msg<<std::endl;
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
void operation_mode_callback(const std_msgs::Int32::ConstPtr &msg) {
	operation_mode = (OPERATION_MODE) (*msg).data;
	current_state = STATES::IDLE;
	change_state = true;

}

void keyboard_callback(const geometry_msgs::Twist::ConstPtr &msg) {
	xref = (*msg).linear.x * 0.01;
	yref = (*msg).linear.y * 0.01;
	zref = (*msg).linear.z * 0.01;

}

void manual_command_callback(const std_msgs::Int32::ConstPtr &msg) {
	manual_command = (STATES) (*msg).data;
}

void force_limiting_callback(const std_msgs::Int32::ConstPtr &msg) {

	force_limiting = (*msg).data;
}

void object_location_update_callback(const std_msgs::Int32::ConstPtr &msg) {
	object_location_update = (*msg).data;
}

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

//	rot2.setEulerZYX(0,-3.14159265/2,0);
//	rot3.setEulerZYX(-3.14159265/2,0,0);
	rot2.setEulerZYX(0, 0, 3.14159265 / 2);
	rot3.setEulerZYX(0, 3.14159265 / 2, 0);

//	rot4=rot2*rot3;//rot3.inverse()*rot2.inverse();
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
	T[0 * 4 + 3] = rot4[0][0] * v1 + rot4[0][1] * v2 + rot4[0][2] * v3; //*/-T[0*4+3];
	T[1 * 4 + 3] = rot4[1][0] * v1 + rot4[1][1] * v2 + rot4[1][2] * v3; //*/-T[1*4+3];
	T[2 * 4 + 3] = rot4[2][0] * v1 + rot4[2][1] * v2 + rot4[2][2] * v3; //*/T[2*4+3];

}
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
	T[0 * 4 + 3] = rot4[0][0] * v1 + rot4[0][1] * v2 + rot4[0][2] * v3; //*/-T[0*4+3];
	T[1 * 4 + 3] = rot4[1][0] * v1 + rot4[1][1] * v2 + rot4[1][2] * v3; //*/-T[1*4+3];
	T[2 * 4 + 3] = rot4[2][0] * v1 + rot4[2][1] * v2 + rot4[2][2] * v3; //*/T[2*4+3];

}

bool go_to_joint(double *joints) {

	control_msgs::JointJog ref;

	ref.joint_names.push_back("shoulder_pan_joint");
	ref.joint_names.push_back("shoulder_lift_joint");
	ref.joint_names.push_back("elbow_joint");
	ref.joint_names.push_back("wrist_1_joint");
	ref.joint_names.push_back("wrist_2_joint");
	ref.joint_names.push_back("wrist_3_joint");
	double gain = 0.4;
	double sum = 0;
	for (int i = 0; i < 6; i++) {
		double diff = joints[i] - jointInfo[i];
		sum = sum + fabs(diff);
		double vel = diff * gain;
		if (fabs(vel) > speed_joint) {
			vel = sign(vel) * speed_joint;
		}
		ref.velocities.push_back(vel);
		std::cout << vel << std::endl;

	}

	servo_reference_joint_publisher.publish(ref);
	if (sum < 0.01) {
		return true;
	}
	return false;

}

bool go_to(double *T_goal) {
	geometry_msgs::Twist ref;
	double T_start[16];
	ur_kinematics::forward(jointInfo, T_start);
	printf("current0\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T_start[i * 4 + j]);
		}
		printf("\n");

	}
	prepare_transformation(T_start);
	/*	printf("\n");
	 for (int i=0;i<6;i++)
	 {
	 printf("%.2f ",jointInfo[i]);
	 }
	 printf("\n\n");*/
	tf::Matrix3x3 rot_start(T_start[0 * 4 + 0], T_start[0 * 4 + 1],
			T_start[0 * 4 + 2], T_start[1 * 4 + 0], T_start[1 * 4 + 1],
			T_start[1 * 4 + 2], T_start[2 * 4 + 0], T_start[2 * 4 + 1],
			T_start[2 * 4 + 2]);
	tf::Matrix3x3 rot_goal(T_goal[0 * 4 + 0], T_goal[0 * 4 + 1],
			T_goal[0 * 4 + 2], T_goal[1 * 4 + 0], T_goal[1 * 4 + 1],
			T_goal[1 * 4 + 2], T_goal[2 * 4 + 0], T_goal[2 * 4 + 1],
			T_goal[2 * 4 + 2]);

	tf::Quaternion q_start, q_goal;
	rot_start.getRotation(q_start);
	rot_goal.getRotation(q_goal);
//	tf::Quaternion q_diff=q_goal-q_start;
	//tf::Matrix3x3 rot_diff(q_diff);
	tf::Matrix3x3 rot_diff(rot_start.inverse() * rot_goal);

	std::cout << "diff rot" << std::endl;
	double yaw, pitch, roll;
//	rot_diff=q_start.inverse()*q_goa	std::cout << ref << std::endl;

	rot_diff.getEulerYPR(yaw, pitch, roll);

	double dx, dy, dz, dist_lin, dist_ang;
	dx = T_goal[0 * 4 + 3] - T_start[0 * 4 + 3];
	dy = T_goal[1 * 4 + 3] - T_start[1 * 4 + 3];
	dz = T_goal[2 * 4 + 3] - T_start[2 * 4 + 3];

	printf("current\n");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.2f ", T_start[i * 4 + j]);
		}
		printf("\n");

	}
	printf("rot\n");
	double dx1, dy1, dz1;

	//dx1=T_goal[0*4+3]*T_start[0*4+0]+T_goal[1*4+3]*T_start[1*4+0]+T_goal[2*4+3]*T_start[2*4+0]+T_start[0*4+3];
	//dy1=T_goal[0*4+3]*T_start[0*4+1]+T_goal[1*4+3]*T_start[1*4+1]+T_goal[2*4+3]*T_start[2*4+1]+T_start[1*4+3];
	//dz1=T_goal[0*4+3]*T_start[0*4+2]+T_goal[1*4+3]*T_start[1*4+2]+T_goal[2*4+3]*T_start[2*4+2]+T_start[2*4+3];

	double current_roll, current_pitch, current_yaw;
	rot_start.getEulerYPR(current_yaw, current_pitch, current_roll);
	printf("current pitch roll yaw %.2f %.2f %.2f %.2f\n ", current_pitch,
			current_roll, current_yaw);

	printf("dx dy dz %.2f %.2f %.2f\n", dx, dy, dz);
	printf("dpitch droll dyaw %.2f %.2f %.2f\n", pitch, roll, yaw);
	if (fabs(roll) > 0.03 || fabs(pitch) > 0.03) {
		//	yaw=0;
	}

	dist_lin = sqrt(dx * dx + dy * dy + dz * dz);
	double scaling = 1;
	if (dist_lin < 0.1)
		scaling = dist_lin / 0.1;
	if (dist_lin < 0.001)
		dist_lin = 0.001;
	//if (dist_lin<0.01) scaling=0;
	dist_ang = sqrt(yaw * yaw + pitch * pitch + roll * roll);
//manual control
	double xref1, yref1, zref1;
	xref1 = -dx / dist_lin * speed_linear * scaling;
	yref1 = -dy / dist_lin * speed_linear * scaling;
	zref1 = dz / dist_lin * speed_linear * scaling;
	ref.linear.x = T_start[0 * 4 + 0] * xref1 + T_start[1 * 4 + 0] * yref1
			+ T_start[2 * 4 + 0] * zref1;//-dx/dist_lin*speed_linear*scaling*0;
	ref.linear.y = T_start[0 * 4 + 1] * xref1 + T_start[1 * 4 + 1] * yref1
			+ T_start[2 * 4 + 1] * zref1;//-dy/dist_lin*speed_linear*scaling*0;
	ref.linear.z = T_start[0 * 4 + 2] * xref1 + T_start[1 * 4 + 2] * yref1
			+ T_start[2 * 4 + 2] * zref1;//dz/dist_lin*speed_linear*scaling*0;
			/*

			 ref.linear.x=dx1/dist_lin*speed_linear*scaling;
			 ref.linear.y=dy1/dist_lin*speed_linear*scaling;
			 ref.linear.z=-dz1/dist_lin*speed_linear*scaling;*/
	if (fabs(ref.linear.x) > speed_linear)
		ref.linear.x = 0;
	if (fabs(ref.linear.y) > speed_linear)
		ref.linear.y = 0;
	if (fabs(ref.linear.z) > speed_linear)
		ref.linear.z = 0;

	double scaling1 = 1;
	if (dist_ang < 0.1)
		scaling1 = dist_ang / 0.1;
	if (dist_ang < 0.001)
		dist_ang = 0.001;
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
	servo_reference_publisher.publish(ref);
	std::cout << "distance " << dist_lin << " " << dist_ang << std::endl;
	if (dist_lin < 0.01 && dist_ang < 0.05)
		return true;
	else
		return false;

}
bool goto_home() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	joint[0] = 180 / 180 * 3.14159265;
	joint[1] = -90. / 180 * 3.14159265;
	joint[2] = -60. / 180 * 3.14159265;
	joint[3] = -80. / 180 * 3.14159265;
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
	return go_to_joint(joint);
}

bool goto_above() {
	sensor_msgs::JointState joints_home;
	double joint[6];
	double above_point1[4][4];
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
	}
	/*above_point[0][0]=-1; above_point[0][1]=0; above_point[0][2]=0;above_point[0][3]=1;
	 above_point[1][0]=0; above_point[1][1]=1; above_point[1][2]=0;above_point[1][3]=1;
	 above_point[2][0]=0;above_point[2][1]=0; above_point[2][2]=-1;above_point[2][3]=1;
	 above_point[3][0]=0; above_point[3][1]=0; above_point[3][2]=0; above_point[3][3]=1;
	 */
	printf("\n above angle %.4f\n", angle_final);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			printf("%.4f ", above_point[i][j]);
			above_point1[i][j] = above_point[i][j];
		}
		printf("\n");
	}
	double sols[8 * 6];
	prepare_transformation_inverse(&above_point1[0][0]);
	int k = ur_kinematics::inverse(&above_point1[0][0], sols, 0);
	int found = 0;
	for (int i = 0; i < k; i++) {
		for (int j = 0; j < 6; j++) {
			if (sols[i * 6 + j] - jointInfo[j] > 3.14159265) {

				sols[i * 6 + j] -= 2 * 3.14159265;
			}
			if (-sols[i * 6 + j] + jointInfo[j] > 3.14159265) {

				sols[i * 6 + j] += 2 * 3.14159265;
			}
			if (j==0)
			{
				if (sols[i*6+j]<0)
				{
					sols[i*6+j]+= 2 * 3.14159265;
				}
			}
			if (j==5)
			{
				if (sols[i*6+j]<-3.14159)
				{
					sols[i*6+j]+=3.14159265;
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
		if (sols[i * 6 + 0] > 135 * 3.14159 / 180
				&& sols[i * 6 + 0] < 225 * 3.14159 / 180) {
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
	if (found == 0) {
		printf("no good solutions\n");
		return false;
	}
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

	return go_to_joint(joint);
	//return go_to(&above_point[0][0]);

}
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

bool scan() {
	static double T_start[16] = { 0 };
	static double joints_start[6] = { 0 };
	double scan_limit = 0.5, increment = -0.2;
	static int scan_line = 0;
	static int increment_line = 0;
	static ros::Time start = ros::Time::now();
	static tf::Quaternion q_start, q_goal;
	static double pitch0 = 0;
	geometry_msgs::Twist ref;
	if (change_state) {

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

	ros::Duration diff = ros::Time::now() - start;
	if (diff.toSec() > 2) {
		double joints[6] = { 0 };
		for (int i = 0; i < 6; i++) {
			joints[i] = joints_start[i];
		}

		if (scan_line % 2 == 0) {
			joints[0] = joints[0] + 0.3;

		} else {
			joints[0] = joints[0] - 0.3;
		}
		joints[3] = joints[3] + pitch0;

		if (go_to_joint(joints)) {
			scan_line = scan_line + 1;
			pitch0 += increment;
		}

	}
	if (fabs(pitch0) > 0.6) {
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
bool goto_drop() {
	stop_movement();
	return true;
	sensor_msgs::JointState joints_home;
	double joint[6];
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
		}
		printf("\n");
	}

	return go_to(&T[0][0]);

}
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
void stop() {
	if (change_state) {
		stop_tool();
	}
	stop_movement();
	change_state = false;
}
bool calculate_above_pose() {
	if (count_boxes > 0) {
		double x_final, y_final, z_final;
		std::sort(x_box.begin(), x_box.end());
		std::sort(y_box.begin(), y_box.end());
		std::sort(z_box.begin(), z_box.end());
		std::sort(angle_box.begin(), angle_box.end());

		x_final = x_box[count_boxes / 2];
		y_final = y_box[count_boxes / 2];
		z_final = z_box[count_boxes / 2] + 0.34;
		angle_final = 6.28 - angle_box[count_boxes / 2] +3.14159/2 ;

		above_point[0][3] = x_final;
		above_point[1][3] = y_final;
		above_point[2][3] = z_final;
		above_point[3][3] = 1;
		//above_point[0][0]=-sqrt(2)/2; above_point[0][1]=sqrt(2)/2; above_point[0][2]=0;
		//above_point[1][0]=sqrt(2)/2; above_point[1][1]=sqrt(2)/2; above_point[1][2]=0;
		//above_point[2][0]=0;above_point[2][1]=0; above_point[2][2]=-1;
		//above_point[3][0]=0; above_point[3][1]=0; above_point[3][2]=0; above_point[3][3]=1;
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
void update() {
	if (operation_mode == OPERATION_MODE::STOP) {
		stop();

	} else {
		switch (current_state) {
		case STATES::IDLE:
			stop();
			if (operation_mode == OPERATION_MODE::AUTO)
				current_state = GOTO_HOME;
			break;
		case STATES::GOTO_HOME:
			if (goto_home()) {
				current_state = STATES::SCAN;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::SCAN:
			if (scan()) {
				if (calculate_above_pose()) {
					current_state = STATES::GOTO_ABOVE;
					if (operation_mode == OPERATION_MODE::MANUAL)
						operation_mode = OPERATION_MODE::STOP;
					change_state = true;
				} else {
					current_state = STATES::GOTO_HOME;
					if (operation_mode == OPERATION_MODE::MANUAL)
						operation_mode = OPERATION_MODE::STOP;
					change_state = true;
				}
			}
			break;
		case STATES::GOTO_ABOVE:

			if (goto_above()) {
				current_state = STATES::SERVO_DOWN;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::SERVO_DOWN:
			if (servo_down()) {
				current_state = STATES::CLOSE_TOOL;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::CLOSE_TOOL:
			if (close_tool()) {
				current_state = STATES::SERVO_UP;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::SERVO_UP:
			if (servo_up()) {
				current_state = STATES::GOTO_DROP;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::GOTO_DROP:
			if (goto_drop()) {
				current_state = STATES::OPEN_TOOL;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				change_state = true;
			}
			break;
		case STATES::OPEN_TOOL:
			if (open_tool()) {
				current_state = STATES::FINISHED;
				if (operation_mode == OPERATION_MODE::MANUAL)
					operation_mode = OPERATION_MODE::STOP;
				stop();
				change_state = true;
			}
			break;
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "ur_control_mbzirc");
	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	gripper_service = nh.serviceClient < ur_msgs::SetIO
			> ("/ur_hardware_interface/set_io");

	int refresh_rate;
	std::string joint_states_topic, force_torque_topic, velocity_cmd_topic =
			"/servo_reference", servoing_topic, operation_mode_topic,
			manual_command_topic, joint_velocity_cmd_topic =
					"/servo_joint_reference", object_location_update_topic,
			servo_force_limiting_topic, box_position_array_topic;
	nh_ns.param("joint_states_topic", joint_states_topic,
			(std::string) "/joint_states");
	nh_ns.param("servoing_topic", servoing_topic,
			(std::string) "/servoing_topic");
	nh_ns.param("operation_mode", operation_mode_topic,
			(std::string) "/operation_mode");
	nh_ns.param("manual_command", manual_command_topic,
			(std::string) "/manual_command");
	nh_ns.param("object_location_update", object_location_update_topic,
			(std::string) "/object_location_update");
	nh_ns.param("servo_force_limiting", servo_force_limiting_topic,
			(std::string) "/servo_force_limiting");

	nh_ns.param("box_position_array_topic", box_position_array_topic,
			(std::string) "/box_position");

	ros::Subscriber joints_subscriber = nh.subscribe(joint_states_topic, 1000,
			joints_callback);

	ros::Subscriber box_position_subscriber = nh.subscribe(
			box_position_array_topic, 1000, box_position_array_callback);

	ros::Subscriber operating_mode_subscriber = nh.subscribe(
			operation_mode_topic, 1000, operation_mode_callback);

	ros::Subscriber keyboard_subscriber = nh.subscribe("/cmd_vel", 1000,
			keyboard_callback);

	ros::Subscriber servo_force_limiting_subscriber = nh.subscribe(
			servo_force_limiting_topic, 1000, force_limiting_callback);

	ros::Subscriber manual_command_subscriber = nh.subscribe(
			manual_command_topic, 1000, manual_command_callback);

	ros::Subscriber object_location_update_subscriber = nh.subscribe(
			object_location_update_topic, 1000,
			object_location_update_callback);

	geometry_msgs::TwistStamped reference;
	servo_reference_publisher = nh.advertise < geometry_msgs::Twist
			> (velocity_cmd_topic, 1000);

	servo_reference_joint_publisher = nh.advertise < control_msgs::JointJog
			> (joint_velocity_cmd_topic, 1000);
	jointInfo[0] = 0;
	jointInfo[1] = 0;
	jointInfo[2] = 0;
	jointInfo[3] = 0;
	jointInfo[4] = 0;
	jointInfo[5] = 0;

	ros::Rate loop_rate(20);
	operation_mode = OPERATION_MODE::AUTO;
	//current_state = STATES::SERVO_DOWN;
	//	current_state=STATES::SCAN;
	while (ros::ok()) {
		if (joints_received == 1) {
			std::cout << "Mode " << current_state << "operation mode "
					<< operation_mode << std::endl;
			//goto_above();
			//servo_up();
			update();
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

