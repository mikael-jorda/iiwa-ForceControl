// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include "tasks/OrientationTask.h"
#include "tasks/HybridPositionTask.h"

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
// const std::string ORI_TASK_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::task_force";
// const std::string ORI_TASK_ERROR_KEY = "sai2::iiwaForceControl::iiwaBot::controller::ori_task::orientation_error";
const std::string POS_TASK_X_DESIRED_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::x_desired";
const std::string POS_TASK_X_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::x";
const std::string POS_TASK_JOINT_TORQUES_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::joint_torques";
// const std::string POS_TASK_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::task_force";
// const std::string POS_TASK_PFORCE_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::force_related_force";
// const std::string POS_TASK_FFORCE_KEY = "sai2::iiwaForceControl::iiwaBot::controller::pos_task::position_related_force";
const std::string FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::force";

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

	// Orientation task
	OrientationTask ori_task = OrientationTask(dof);
	ori_task.link_name = "link6";
	ori_task.kp = 500.0;
	ori_task.kv = 70.0;

	Eigen::VectorXd ori_task_torques(dof);
	Eigen::Matrix3d initial_orientation;
	robot->rotation(initial_orientation, ori_task.link_name);
	ori_task.desired_orientation = initial_orientation;
	ori_task.desired_orientation = initial_orientation;

	// operational space position task
	HybridPositionTask pos_task = HybridPositionTask(dof);
	pos_task.link_name = "link6";
	pos_task.pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
	pos_task.kp = 100.0;
	pos_task.kv = 20.0;

	Eigen::VectorXd pos_task_torques(dof);
	Eigen::Vector3d initial_position;
	robot->position(initial_position,pos_task.link_name,pos_task.pos_in_link);
	pos_task.current_position = initial_position;
	pos_task.desired_position = initial_position;

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

	Eigen::Vector3d sensed_force = Eigen::Vector3d::Zero();

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getCommandIs(FORCE_SENSOR_FORCE_KEY, sensed_force(2));

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
			// N_tmp = N_prec;

			// position controller 
			robot->Jv(pos_task.jacobian,pos_task.link_name,pos_task.pos_in_link);
			pos_task.projected_jacobian = pos_task.jacobian * N_prec;
			robot->operationalSpaceMatrices(pos_task.Lambda, pos_task.Jbar, pos_task.N,
									pos_task.projected_jacobian, N_prec);
			N_prec = pos_task.N;

		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//---- orientation controller
		// update current orientation
		robot->rotation(ori_task.current_orientation, ori_task.link_name);
		ori_task.current_angular_velocity = ori_task.projected_jacobian*robot->_dq;
		// update desired orientation
		Eigen::Matrix3d ori_update;
		// if (controller_counter >= 3000)
		// {
		// 	double fr = 0.015*M_PI/180;
		// 	ori_update << 1, 0, 0,
		// 			  0, cos(fr), -sin(fr),
		// 			  0, sin(fr), cos(fr);
		// 	// ori_task.desired_orientation = ori_update * ori_task.desired_orientation;
		// }
		redis_client.setEigenMatrixDerivedString(ORI_TASK_R_KEY, ori_task.current_orientation);
		redis_client.setEigenMatrixDerivedString(ORI_TASK_R_DESIRED_KEY, ori_task.desired_orientation);
		// compute torques
		ori_task.computeTorques(ori_task_torques);
		redis_client.setEigenMatrixDerivedString(ORI_TASK_JOINT_TORQUES_KEY, ori_task_torques);

		// sensed_force = ori_task.current_orientation*sensed_force;

		//---- position controller 
		// find current position
		robot->position(pos_task.current_position, pos_task.link_name, pos_task.pos_in_link);
		pos_task.current_velocity = pos_task.projected_jacobian * robot->_dq;
		// update desired position
		// double freq = 0.25;
		// double amplitude = 0.1;
		// pos_task.desired_position = initial_position + amplitude/2*Eigen::Vector3d(cos(2*M_PI*freq*time)-1,sin(2*M_PI*freq*time),0);
		// pos_task.desired_position = initial_position;
		if(controller_counter < 3000)
		// if(sensed_force(2) > -5)
		{
			pos_task.desired_position(2) -= 0.00016;
		}
		if(controller_counter % 1000 == 0)
		{
			std::cout << sensed_force.transpose() << std::endl;
		}

		if (controller_counter == 4000)
		// if (sensed_force(2) < -50)
		{
			pos_task.desired_force = Eigen::Vector3d(0.0,0.0,-5);
			pos_task.setForceAxis(Eigen::Vector3d(0,0,1));
		}

		if(controller_counter >= 6000)
		{
			pos_task.desired_force(2) = -5 - 1.0*(sensed_force(2)+5);
		}

		// compute joint torques
		pos_task.computeTorques(pos_task_torques);

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = pos_task_torques + ori_task_torques + N_prec.transpose()*joint_task_torques;

		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);
    fgc_command_enabled = "0";
    redis_client.setCommandIs(FGC_ENABLE_KEY,fgc_command_enabled);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
