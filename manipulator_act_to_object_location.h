#pragma once
#include "stdafx.h"
#include "rsdef.h"
#include <string>
#include "example.h"

//虚拟机地址
//#define ROBOT_ADDR "192.168.56.129"

//机械臂地址
#define ROBOT_ADDR "192.168.1.233"
//#define ROBOT_ADDR "192.168.65.131"

#define ROBOT_PORT 8899

//机械臂控制上下文句柄
//RSHD g_rshd = -1;
RSHD g_rshd = 1;
#define M_PI 3.1415926

typedef struct {
	double x;
	double y;
	double z;
}user_original_onbase;

typedef struct {
	double x = 0;
	double y = 0;
	double z = 0;
}user_point_position;

typedef struct {
	double rx = 0;
	double ry = 0;
	double rz = 0;
}_rpy;

inline int manipulator_act_to_object_location(double tgt_x_onuser,double tgt_y_onuser,double tgt_z_onuser,float alpha,float phi,float rotate_z) {

	//登录服务器
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{

		Sleep(15000);


		//用户坐标系原点位置
		user_original_onbase user_original_onbase1;
		user_original_onbase1.x = -0.600319;
		user_original_onbase1.y = -0.131499;
		user_original_onbase1.z = 1.01;

		//用户坐标系的目标位置
		user_point_position user_position_onuser;
	/*	user_position_onuser.x = 0.0;
		user_position_onuser.y = 0.0;
		user_position_onuser.z = 0.4;*/
		user_position_onuser.x = tgt_x_onuser;
		user_position_onuser.y = tgt_y_onuser;
		user_position_onuser.z = tgt_z_onuser;
		


		//用户坐标系的点转换到base坐标系下的坐标
		user_point_position user_position_onbase;
		user_position_onbase.x = user_original_onbase1.x - user_position_onuser.y;
		user_position_onbase.y = user_original_onbase1.y - user_position_onuser.x;
		user_position_onbase.z = user_original_onbase1.z - user_position_onuser.z;

		aubo_robot_namespace::Rpy user_rpy_onbase;
		aubo_robot_namespace::Ori user_ori_onbase;
		//基础姿态
		user_rpy_onbase.rx = 0;//R
		user_rpy_onbase.ry = M_PI;//p
		user_rpy_onbase.rz = M_PI / 2;//Y  若要选装末端坐标系的z轴正转，则要减去相应的角度
		//点云所求得的姿态
		user_rpy_onbase.rx -= alpha;//R
		user_rpy_onbase.ry -= phi;//p
		user_rpy_onbase.rz += rotate_z;







		double phi = user_rpy_onbase.rx;
		double theta = user_rpy_onbase.ry;
		double psi = user_rpy_onbase.rz;


		//欧拉角的姿态转变为四元素形式
		user_ori_onbase.w = cos(phi / 2)*cos(theta / 2)*cos(psi / 2) + sin(phi / 2)*sin(theta / 2)*sin(psi / 2);
		user_ori_onbase.x = sin(phi / 2)*cos(theta / 2)*cos(psi / 2) - cos(phi / 2)*sin(theta / 2)*sin(psi / 2);
		user_ori_onbase.y = cos(phi / 2)*sin(theta / 2)*cos(psi / 2) + sin(phi / 2)*cos(theta / 2)*sin(psi / 2);
		user_ori_onbase.z = cos(phi / 2)*cos(theta / 2)*sin(psi / 2) - sin(phi / 2)*sin(theta / 2)*cos(psi / 2);
		//aubo自带的rs_rpy_to_quaternion不能使用
		cout << "user_rpy_onbase.rx" << user_rpy_onbase.rx << endl << user_rpy_onbase.rz << endl;
		//rs_rpy_to_quaternion(g_rshd, &user_rpy_onbase, &user_ori_onbase);//欧拉角转四元素
		cout << "user_ori_onbase:  " << user_ori_onbase.w << "  " << user_ori_onbase.x << "  " << user_ori_onbase.y << "  " << user_ori_onbase.z << endl;

//实验rs_rpy_tp_quaternion()
		//aubo_robot_namespace::Ori user_ori_onbase_test;
		//rs_rpy_to_quaternion(g_rshd, &user_rpy_onbase, &user_ori_onbase_test);
		//cout << "user_ori_onbase_test:  " << user_ori_onbase_test.w << "  " 
		//	<< user_ori_onbase_test.x << "  " << user_ori_onbase_test.y
		//	<< "  " << user_ori_onbase_test.z << endl;





		//启动机械臂(必须连接真实机械臂）
		//example_robotStartup(g_rshd);
		rs_set_global_end_max_line_velc(g_rshd, 0.05);

		//吸盘吸取
		rs_set_board_io_status_by_name(g_rshd, RobotBoardUserDO, "U_DO_00", 1.0);

		////机械臂轴动测试
		example_moveJ(g_rshd);

		////机械臂保持当前姿态直线运动测试
		//example_moveL(g_rshd);

		//机械臂轨迹运动测试
		//example_moveP(g_rshd);

		////机械臂正逆解测试
		//example_ik_fk(g_rshd);
		aubo_robot_namespace::wayPoint_S curway, tgt_point, inverse_msg;
		//base下的目标位置和姿态
		tgt_point.cartPos.position.x = user_position_onbase.x;
		tgt_point.cartPos.position.y = user_position_onbase.y;
		tgt_point.cartPos.position.z = user_position_onbase.z;
		tgt_point.orientation = user_ori_onbase;

		//计算当前点移动到目标点的路径
		rs_get_current_waypoint(g_rshd, &curway);
		rs_inverse_kin(g_rshd, curway.jointpos, &tgt_point.cartPos.position, &tgt_point.orientation, &inverse_msg);
		double translate_Radian[6] = { 0 };
		translate_Radian[0] = inverse_msg.jointpos[0];
		translate_Radian[1] = inverse_msg.jointpos[1];
		translate_Radian[2] = inverse_msg.jointpos[2];
		translate_Radian[3] = inverse_msg.jointpos[3];
		translate_Radian[4] = inverse_msg.jointpos[4];
		translate_Radian[5] = inverse_msg.jointpos[5];


		bool isBlock = true;
		rs_move_joint(g_rshd, translate_Radian, isBlock);

	
		//rs_move_pause(g_rshd);
		/*clock_t wait_tool_action_start, wait_tool_action_end;
		wait_tool_action_start = clock();
		wait_tool_action_end = wait_tool_action_start + 10000;
		if (wait_tool_action_end == clock())*/

		//	rs_move_continue(g_rshd);
		Sleep(10000);
		
		//提升
		
		aubo_robot_namespace::wayPoint_S elevate_point,curway1, inverse_msg1;
		rs_get_current_waypoint(g_rshd, &curway1);
		elevate_point = curway1;
		elevate_point.cartPos.position.z += 0.1;
		rs_inverse_kin(g_rshd, curway1.jointpos, &elevate_point.cartPos.position, &elevate_point.orientation, &inverse_msg1);
		double elevate_Radian[6] = { 0 };
		elevate_Radian[0] = inverse_msg1.jointpos[0];
		elevate_Radian[1] = inverse_msg1.jointpos[1];
		elevate_Radian[2] = inverse_msg1.jointpos[2];
		elevate_Radian[3] = inverse_msg1.jointpos[3];
		elevate_Radian[4] = inverse_msg1.jointpos[4];
		elevate_Radian[5] = inverse_msg1.jointpos[5];

		rs_move_joint(g_rshd, elevate_Radian, isBlock);




		//左后移动一段
		aubo_robot_namespace::wayPoint_S left_right_point, curway2, inverse_msg2;
		rs_get_current_waypoint(g_rshd, &curway2);
		left_right_point = curway2;
		left_right_point.cartPos.position.x += 0.50;
		left_right_point.cartPos.position.y -= 0.4;

		rs_inverse_kin(g_rshd, curway2.jointpos, &left_right_point.cartPos.position, &left_right_point.orientation, &inverse_msg2);
		double left_right_Radian[6] = { 0 };
		left_right_Radian[0] = inverse_msg2.jointpos[0];
		left_right_Radian[1] = inverse_msg2.jointpos[1];
		left_right_Radian[2] = inverse_msg2.jointpos[2];
		left_right_Radian[3] = inverse_msg2.jointpos[3];
		left_right_Radian[4] = inverse_msg2.jointpos[4];
		left_right_Radian[5] = inverse_msg2.jointpos[5];

		rs_move_joint(g_rshd, left_right_Radian, isBlock);

		//下降
		aubo_robot_namespace::wayPoint_S down_point, curway3, inverse_msg3;
		rs_get_current_waypoint(g_rshd, &curway3);
		down_point = curway3;
		down_point.cartPos.position.z -= 0.08;

		rs_inverse_kin(g_rshd, curway3.jointpos, &down_point.cartPos.position, &down_point.orientation, &inverse_msg3);
		double down_Radian[6] = { 0 };
		down_Radian[0] = inverse_msg3.jointpos[0];
		down_Radian[1] = inverse_msg3.jointpos[1];
		down_Radian[2] = inverse_msg3.jointpos[2];
		down_Radian[3] = inverse_msg3.jointpos[3];
		down_Radian[4] = inverse_msg3.jointpos[4];
		down_Radian[5] = inverse_msg3.jointpos[5];

		rs_move_joint(g_rshd, down_Radian, isBlock);

		rs_set_board_io_status_by_name(g_rshd, RobotBoardUserDO, "U_DO_00", 0.0);


		//退出登录
		example_logout(g_rshd);
	}

	//反初始化接口库
	rs_uninitialize();

	std::cout << "please enter to exit" << std::endl;
	getchar();

	return 0;


}



inline int manipulator_act_to_object_location(double tgt_x_onuser, double tgt_y_onuser, double tgt_z_onuser) {

	//登录服务器
	if (example_login(g_rshd, ROBOT_ADDR, ROBOT_PORT))
	{


		//用户坐标系原点位置
		user_original_onbase user_original_onbase1;
		user_original_onbase1.x = -0.600319;
		user_original_onbase1.y = -0.131499;
		user_original_onbase1.z = 1.01;

		//用户坐标系的目标位置
		user_point_position user_position_onuser;
		/*	user_position_onuser.x = 0.0;
		user_position_onuser.y = 0.0;
		user_position_onuser.z = 0.4;*/
		user_position_onuser.x = tgt_x_onuser;
		user_position_onuser.y = tgt_y_onuser;
		user_position_onuser.z = tgt_z_onuser;



		//用户坐标系的点转换到base坐标系下的坐标
		user_point_position user_position_onbase;
		user_position_onbase.x = user_original_onbase1.x - user_position_onuser.y;
		user_position_onbase.y = user_original_onbase1.y - user_position_onuser.x;
		user_position_onbase.z = user_original_onbase1.z - user_position_onuser.z;

		aubo_robot_namespace::Rpy user_rpy_onbase;
		aubo_robot_namespace::Ori user_ori_onbase;
		user_rpy_onbase.rx = 0;//R
		user_rpy_onbase.ry = M_PI;//p
		user_rpy_onbase.rz = M_PI / 2;//Y  若要选装末端坐标系的z轴正转，则要减去相应的角度






		double phi = user_rpy_onbase.rx;
		double theta = user_rpy_onbase.ry;
		double psi = user_rpy_onbase.rz;


		//欧拉角的姿态转变为四元素形式
		user_ori_onbase.w = cos(phi / 2)*cos(theta / 2)*cos(psi / 2) + sin(phi / 2)*sin(theta / 2)*sin(psi / 2);
		user_ori_onbase.x = sin(phi / 2)*cos(theta / 2)*cos(psi / 2) - cos(phi / 2)*sin(theta / 2)*sin(psi / 2);
		user_ori_onbase.y = cos(phi / 2)*sin(theta / 2)*cos(psi / 2) + sin(phi / 2)*cos(theta / 2)*sin(psi / 2);
		user_ori_onbase.z = cos(phi / 2)*cos(theta / 2)*sin(psi / 2) - sin(phi / 2)*sin(theta / 2)*cos(psi / 2);
		//aubo自带的rs_rpy_to_quaternion不能使用
		cout << "user_rpy_onbase.rx" << user_rpy_onbase.rx << endl << user_rpy_onbase.rz << endl;
		//rs_rpy_to_quaternion(g_rshd, &user_rpy_onbase, &user_ori_onbase);//欧拉角转四元素
		cout << "user_ori_onbase:  " << user_ori_onbase.w << "  " << user_ori_onbase.x << "  " << user_ori_onbase.y << "  " << user_ori_onbase.z << endl;

		//实验rs_rpy_tp_quaternion()
		//aubo_robot_namespace::Ori user_ori_onbase_test;
		//rs_rpy_to_quaternion(g_rshd, &user_rpy_onbase, &user_ori_onbase_test);
		//cout << "user_ori_onbase_test:  " << user_ori_onbase_test.w << "  " 
		//	<< user_ori_onbase_test.x << "  " << user_ori_onbase_test.y
		//	<< "  " << user_ori_onbase_test.z << endl;





		//启动机械臂(必须连接真实机械臂）
		//example_robotStartup(g_rshd);
		rs_set_global_end_max_line_velc(g_rshd, 0.05);

		//吸盘吸取
		rs_set_board_io_status_by_name(g_rshd, RobotBoardUserDO, "U_DO_00", 1.0);

		////机械臂轴动测试
		example_moveJ(g_rshd);

		////机械臂保持当前姿态直线运动测试
		//example_moveL(g_rshd);

		//机械臂轨迹运动测试
		//example_moveP(g_rshd);

		////机械臂正逆解测试
		//example_ik_fk(g_rshd);
		aubo_robot_namespace::wayPoint_S curway, tgt_point, inverse_msg;
		//base下的目标位置和姿态
		tgt_point.cartPos.position.x = user_position_onbase.x;
		tgt_point.cartPos.position.y = user_position_onbase.y;
		tgt_point.cartPos.position.z = user_position_onbase.z;
		tgt_point.orientation = user_ori_onbase;

		//计算当前点移动到目标点的路径
		rs_get_current_waypoint(g_rshd, &curway);
		rs_inverse_kin(g_rshd, curway.jointpos, &tgt_point.cartPos.position, &tgt_point.orientation, &inverse_msg);
		double translate_Radian[6] = { 0 };
		translate_Radian[0] = inverse_msg.jointpos[0];
		translate_Radian[1] = inverse_msg.jointpos[1];
		translate_Radian[2] = inverse_msg.jointpos[2];
		translate_Radian[3] = inverse_msg.jointpos[3];
		translate_Radian[4] = inverse_msg.jointpos[4];
		translate_Radian[5] = inverse_msg.jointpos[5];


		bool isBlock = true;
		rs_move_joint(g_rshd, translate_Radian, isBlock);


		//rs_move_pause(g_rshd);
		/*clock_t wait_tool_action_start, wait_tool_action_end;
		wait_tool_action_start = clock();
		wait_tool_action_end = wait_tool_action_start + 10000;
		if (wait_tool_action_end == clock())*/

		//	rs_move_continue(g_rshd);
		Sleep(10000);

		//提升

		aubo_robot_namespace::wayPoint_S elevate_point, curway1, inverse_msg1;
		rs_get_current_waypoint(g_rshd, &curway1);
		elevate_point = curway1;
		elevate_point.cartPos.position.z += 0.1;
		rs_inverse_kin(g_rshd, curway1.jointpos, &elevate_point.cartPos.position, &elevate_point.orientation, &inverse_msg1);
		double elevate_Radian[6] = { 0 };
		elevate_Radian[0] = inverse_msg1.jointpos[0];
		elevate_Radian[1] = inverse_msg1.jointpos[1];
		elevate_Radian[2] = inverse_msg1.jointpos[2];
		elevate_Radian[3] = inverse_msg1.jointpos[3];
		elevate_Radian[4] = inverse_msg1.jointpos[4];
		elevate_Radian[5] = inverse_msg1.jointpos[5];

		rs_move_joint(g_rshd, elevate_Radian, isBlock);




		//左后移动一段
		aubo_robot_namespace::wayPoint_S left_right_point, curway2, inverse_msg2;
		rs_get_current_waypoint(g_rshd, &curway2);
		left_right_point = curway2;
		left_right_point.cartPos.position.x += 0.1;
		left_right_point.cartPos.position.y -= 0.2;

		rs_inverse_kin(g_rshd, curway2.jointpos, &left_right_point.cartPos.position, &left_right_point.orientation, &inverse_msg2);
		double left_right_Radian[6] = { 0 };
		left_right_Radian[0] = inverse_msg2.jointpos[0];
		left_right_Radian[1] = inverse_msg2.jointpos[1];
		left_right_Radian[2] = inverse_msg2.jointpos[2];
		left_right_Radian[3] = inverse_msg2.jointpos[3];
		left_right_Radian[4] = inverse_msg2.jointpos[4];
		left_right_Radian[5] = inverse_msg2.jointpos[5];

		rs_move_joint(g_rshd, left_right_Radian, isBlock);

		//下降
		aubo_robot_namespace::wayPoint_S down_point, curway3, inverse_msg3;
		rs_get_current_waypoint(g_rshd, &curway3);
		down_point = curway3;
		down_point.cartPos.position.z -= 0.05;

		rs_inverse_kin(g_rshd, curway3.jointpos, &down_point.cartPos.position, &down_point.orientation, &inverse_msg3);
		double down_Radian[6] = { 0 };
		down_Radian[0] = inverse_msg3.jointpos[0];
		down_Radian[1] = inverse_msg3.jointpos[1];
		down_Radian[2] = inverse_msg3.jointpos[2];
		down_Radian[3] = inverse_msg3.jointpos[3];
		down_Radian[4] = inverse_msg3.jointpos[4];
		down_Radian[5] = inverse_msg3.jointpos[5];

		rs_move_joint(g_rshd, down_Radian, isBlock);

		rs_set_board_io_status_by_name(g_rshd, RobotBoardUserDO, "U_DO_00", 0.0);

















		//退出登录
		example_logout(g_rshd);
	}

	//反初始化接口库
	rs_uninitialize();

	std::cout << "please enter to exit" << std::endl;
	getchar();

	return 0;


}