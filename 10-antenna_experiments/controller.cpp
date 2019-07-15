// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "resources/iiwa7.urdf";

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

// - interface
const string CONTROLLER_TYPE_KEY = "sai2::examples::primitive";

const string DESIRED_POSITION_KEY = "sai2::iiwaForceControl::controller::interface::posori_task::desired_position";
const string DESIRED_JOINT_POSITION_KEY = "sai2::iiwaForceControl::controller::interface::joint_task::desired_position";

const string JOINT_KP_KEY = "sai2::iiwaForceControl::controller::joint_kp";
const string JOINT_KV_KEY = "sai2::iiwaForceControl::controller::joint_kv";
const string POS_KP_KEY = "sai2::iiwaForceControl::controller::pos_kp";
const string POS_KV_KEY = "sai2::iiwaForceControl::controller::pos_kv";
const string ORI_KP_KEY = "sai2::iiwaForceControl::controller::ori_kp";
const string ORI_KV_KEY = "sai2::iiwaForceControl::controller::ori_kv";

const string MAX_JOINT_VELOCITY_KEY = "sai2::iiwaForceControl::controller::joint_interpolation_max_velocity";

#define   JOINT_CONTROL       0
#define   CARTESIAN_CONTROL   1

int state = JOINT_CONTROL;
int previous_state = state;

unsigned long long controller_counter = 0;
const bool flag_simulation = true;
// const bool flag_simulation = false;

int main() {

	if(flag_simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";

		JOINT_TORQUES_SENSED_KEY = "sai2::KUKA_IIWA::sensors::torques";
		MASSMATRIX_KEY = "sai2::KUKA_IIWA::model::massmatrix";
		MASSMATRIX_KEY = "sai2::KUKA_IIWA::model::coriolis";
		MASSMATRIX_KEY = "sai2::KUKA_IIWA::model::robot_gravity";
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
	robot->updateModel();

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	VectorXd coriolis = VectorXd::Zero(dof);

	// posori task
	string control_link = "link7";
	Affine3d control_frame = Affine3d::Identity();
	control_frame.translation() = Vector3d(0.0, 0.0, 0.1);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_frame);

	VectorXd posori_task_torques = VectorXd::Zero(dof);

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	VectorXd joint_task_torques = VectorXd::Zero(dof);

	double joint_max_vel = M_PI/3;

	joint_task->_otg->setMaxVelocity(joint_max_vel);
	joint_task->_otg->setMaxAcceleration(3*M_PI);
	joint_task->_otg->setMaxJerk(15*M_PI);

	// set interface
	redis_client.set(JOINT_KP_KEY, to_string(joint_task->_kp));
	redis_client.set(JOINT_KV_KEY, to_string(joint_task->_kv));
	redis_client.set(POS_KP_KEY, to_string(posori_task->_kp_pos));
	redis_client.set(POS_KV_KEY, to_string(posori_task->_kv_pos));
	redis_client.set(ORI_KP_KEY, to_string(posori_task->_kp_ori));
	redis_client.set(ORI_KV_KEY, to_string(posori_task->_kv_ori));

	redis_client.setEigenMatrixJSON(DESIRED_POSITION_KEY, posori_task->_desired_position);
	redis_client.setEigenMatrixJSON(DESIRED_JOINT_POSITION_KEY, joint_task->_desired_position);

	redis_client.set(CONTROLLER_TYPE_KEY, to_string(state));

	redis_client.set(MAX_JOINT_VELOCITY_KEY, to_string(joint_max_vel));

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// while window is open:
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// read from interface
		joint_task->_kp = stod(redis_client.get(JOINT_KP_KEY));
		joint_task->_kv = stod(redis_client.get(JOINT_KV_KEY));
		posori_task->_kp_pos = stod(redis_client.get(POS_KP_KEY));
		posori_task->_kv_pos = stod(redis_client.get(POS_KV_KEY));
		posori_task->_kp_ori = stod(redis_client.get(ORI_KP_KEY));
		posori_task->_kv_ori = stod(redis_client.get(ORI_KV_KEY));

		joint_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_JOINT_POSITION_KEY);
		posori_task->_desired_position = redis_client.getEigenMatrixJSON(DESIRED_POSITION_KEY);

		joint_max_vel = stod(redis_client.get(MAX_JOINT_VELOCITY_KEY));
		joint_task->_otg->setMaxVelocity(joint_max_vel);


		state = stoi(redis_client.get(CONTROLLER_TYPE_KEY));
		if(state != 0)
		{
			if(state != 1)
			{
				state = previous_state;
				redis_client.set(CONTROLLER_TYPE_KEY, to_string(state));
			}
		}
		if(state != previous_state)
		{
			joint_task->reInitializeTask();
			posori_task->reInitializeTask();
		}


		// update model
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			robot->_M_inv = robot->_M.inverse();
		}

		if(state == JOINT_CONTROL)
		{
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			for(int i=0 ; i<7 ; i++)
			{
				robot->_M(i,i) += 0.1;
			}

			joint_task->computeTorques(joint_task_torques);
			
			command_torques = joint_task_torques;
		}

		else if(state == CARTESIAN_CONTROL)
		{
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			for(int i=3 ; i<6 ; i++)
			{
				posori_task->_Lambda(i,i) += 0.1;
			}

			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
		}

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
		}

		previous_state = state;
		controller_counter++;
	}

    command_torques.setZero();
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
