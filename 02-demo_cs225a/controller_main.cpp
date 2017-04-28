// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include "tasks/HybridPositionTask.h"

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string world_file = "resources/02-demo_cs225a/world.urdf";
const string robot_file = "../robot_models/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
const std::string FGC_ENABLE_KEY  = "sai2::iiwaForceControl::iiwaBot::fgc_command_enabled";

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
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl_kuka, Model::urdf, true);

	// read from Redis
	redis_client.getEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);

	Eigen::MatrixXd N_prec;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);

	double joint_kv = 20.0;

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

			// position controller 
			robot->Jv(pos_task.jacobian,pos_task.link_name,pos_task.pos_in_link);
			pos_task.projected_jacobian = pos_task.jacobian * N_prec;
			robot->operationalSpaceMatrices(pos_task.Lambda, pos_task.Jbar, pos_task.N,
									pos_task.projected_jacobian, N_prec);
			N_prec = pos_task.N;
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//---- position controller 
		// find current position
		robot->position(pos_task.current_position, pos_task.link_name, pos_task.pos_in_link);
		pos_task.current_velocity = pos_task.projected_jacobian * robot->_dq;
		// update desired position
		double freq = 0.25;
		double amplitude = 0.1;
		pos_task.desired_position = initial_position + amplitude/2*Eigen::Vector3d(cos(2*M_PI*freq*time)-1,sin(2*M_PI*freq*time),0);
		// pos_task.desired_position = initial_position;

		// compute joint torques
		pos_task.computeTorques(pos_task_torques);

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = pos_task_torques + N_prec.transpose()*joint_task_torques;

		redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

    return 0;
}
