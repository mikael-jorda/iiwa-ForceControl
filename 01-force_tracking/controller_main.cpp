// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include "tasks/OrientationTask.h"

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string world_file = "resources/world.urdf";
const string robot_file = "../robot_models/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;
std::string fgc_command_enabled = "";

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
const std::string FGC_ENABLE_KEY  = "sai2::iiwaForceControl::iiwaBot::fgc_command_enabled";
// - debug
const std::string ORI_TASK_R_DESIRED_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::R_desired";
const std::string ORI_TASK_R_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::R";
const std::string ORI_TASK_JOINT_TORQUES_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::joint_torques";
// const std::string ORI_TASK_ERROR_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::orientation_error";

 void sighandler(int sig)
 { runloop = false; }

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	HiredisServerInfo info;
	info.hostname_ = "127.0.0.1";
	info.port_ = 6379;
	info.timeout_ = { 1, 500000 }; // 1.5 seconds
	auto redis_client = CDatabaseRedisClient();
	redis_client.serverIs(info);

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// read from Redis
	redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::MatrixXd N_prec, N_tmp;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);

	double joint_kv = 20.0;

	// operational space position task
	std::string op_pos_task_link_name = "link6";
	Eigen::Vector3d op_pos_task_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
	Eigen::Vector3d op_pos_task_x, op_pos_task_dx, op_pos_task_x_desired, op_pos_task_f;
	Eigen::MatrixXd op_pos_task_jacobian(3,dof), op_pos_task_J(3,dof);
	Eigen::MatrixXd Lambda_pos(3,3);
	// Eigen::MatrixXd op_pos_task_N(dof,dof);
	Eigen::VectorXd op_pos_task_torques(dof);

	double op_pos_kp = 100.0;
	double op_pos_kv = 20.0;

	Eigen::Vector3d initial_position;
	robot->position(initial_position,op_pos_task_link_name,op_pos_task_pos_in_link);

	// Orientation task
	OrientationTask ori_task = OrientationTask(dof);
	ori_task.link_name = "link6";
	ori_task.kp = 100.0;
	ori_task.kv = 20.0;

	Eigen::VectorXd ori_task_torques(dof);
	Eigen::Matrix3d initial_orientation;
	robot->rotation(initial_orientation, ori_task.link_name);
	ori_task.desired_orientation = initial_orientation;
	ori_task.desired_orientation = ori_task.current_orientation;

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// enable the controller
	fgc_command_enabled = "1";
	redis_client.setCommandIs(FGC_ENABLE_KEY,fgc_command_enabled);

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			/////////////////////////// update jacobians, mass matrices and nullspaces
			N_prec = Eigen::MatrixXd::Identity(dof,dof);
			N_tmp = Eigen::MatrixXd::Identity(dof,dof);

			// orientation controller
			robot->Jw(ori_task.jacobian, ori_task.link_name);
			ori_task.projected_jacobian = ori_task.jacobian * N_prec;
			robot->operationalSpaceMatrices(ori_task.Lambda, ori_task.Jbar, ori_task.N,
									ori_task.projected_jacobian, N_prec);
			N_prec = ori_task.N;
			N_tmp = N_prec;

			// position controller 
			robot->Jv(op_pos_task_jacobian,op_pos_task_link_name,op_pos_task_pos_in_link);
			op_pos_task_J = op_pos_task_jacobian*N_prec;
			robot->taskInertiaMatrix(Lambda_pos,op_pos_task_J);
			robot->nullspaceMatrix(N_prec,op_pos_task_J,N_tmp);
			N_tmp = N_prec;

		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//---- position controller 
		// find current position
		robot->position(op_pos_task_x,op_pos_task_link_name,op_pos_task_pos_in_link);
		op_pos_task_dx = op_pos_task_J*robot->_dq;
		// update desired position
		double freq = 0.25;
		double amplitude = 0.2;
		// op_pos_task_x_desired = initial_position + amplitude/2*Eigen::Vector3d(cos(2*M_PI*freq*time)-1,sin(2*M_PI*freq*time),0);
		op_pos_task_x_desired = initial_position;
		// compute joint torques
		op_pos_task_f = Lambda_pos*(-op_pos_kp*(op_pos_task_x - op_pos_task_x_desired) - op_pos_kv*op_pos_task_dx);
		op_pos_task_torques = op_pos_task_J.transpose()*op_pos_task_f;

		//---- orientation controller
		// update current orientation
		robot->rotation(ori_task.current_orientation, ori_task.link_name);
		ori_task.current_angular_velocity = ori_task.projected_jacobian*robot->_dq;
		// update desired orientation
		Eigen::Matrix3d ori_update;
		if (controller_counter >= 3000)
		{
			double fr = 0.015*M_PI/180;
			ori_update << 1, 0, 0,
					  0, cos(fr), -sin(fr),
					  0, sin(fr), cos(fr);
			ori_task.desired_orientation = ori_update * ori_task.desired_orientation;
		}
		redis_client.setEigenMatrixDerivedString(ORI_TASK_R_KEY, ori_task.current_orientation);
		redis_client.setEigenMatrixDerivedString(ORI_TASK_R_DESIRED_KEY, ori_task.desired_orientation);
		// compute torques
		ori_task.computeTorques(ori_task_torques);
		redis_client.setEigenMatrixDerivedString(ORI_TASK_JOINT_TORQUES_KEY, ori_task_torques);

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = op_pos_task_torques + ori_task_torques + N_prec.transpose()*joint_task_torques;

		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    fgc_command_enabled = "0";
    redis_client.setCommandIs(FGC_ENABLE_KEY,fgc_command_enabled);

    return 0;
}
