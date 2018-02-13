// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

// #include "tasks/OrientationTask.h"
// #include "tasks/HybridPositionTask.h"
// #include "tasks/TestTask.h"
#include "tasks/PosOriTask.h"

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

#define INITIAL                10
#define MOVE_ABOVE_CONTACT     0
#define ZERO_FORCE_SENSOR      1
#define GO_TO_CONTACT          2
#define OL_FORCE_CONTROL       3
#define CL_FORCE_CONTROL       4

// using namespace std;
int state;

const string world_file = "../resources/00-passivity_tests/world.urdf";
const string robot_file = "../resources/00-passivity_tests/kuka_iiwa.urdf";
const std::string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

const bool simulation = true;
// const bool simulation = false;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;
const std::string MASSMATRIX_KEY = "sai2::KUKA_IIWA::sensors::massmatrix";

// - data log
const std::string EE_DESIRED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::desired_force";
const std::string EE_SENSED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::sensed_force";
const std::string RC_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::Rc";

void sighandler(int sig)
{ runloop = false; }

int main() {
	if(simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force_moment";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
		EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";
	}

	std::cout << "Loading URDF world model file: " << world_file << std::endl;

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
	auto robot = new Sai2Model::Sai2Model(robot_file, false);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::MatrixXd N_prec;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(dof), joint_task_torques(dof);

	double joint_kv = 1.0;

	// position and orientation task
	const string control_link = "link6";
	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.translation() = Eigen::Vector3d(0.0, 0.0, 0.12);
	Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity();
	sensor_frame.translation() = Eigen::Vector3d(0.0, 0.0, 0.02);

	auto posori_task = new PosOriTask(robot, "link6", control_frame, sensor_frame);

	posori_task->_kp_ori = 600.0;
	posori_task->_kv_ori = 50.0;
	posori_task->_kp_pos = 150.0;
	posori_task->_kv_pos = 22.0;

	Eigen::VectorXd posori_task_torques(dof);

	// ------------  unnecessary
	Eigen::Matrix3d initial_orientation;
	robot->rotation(initial_orientation, control_link);
	posori_task->_current_orientation = initial_orientation;
	posori_task->_desired_orientation = initial_orientation;

	Eigen::Vector3d initial_position;
	robot->position(initial_position, posori_task->_link_name, posori_task->_T_link_control.translation());
	posori_task->_current_position = initial_position;
	posori_task->_desired_position = initial_position;
	// ------------------------------

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd sensed_force_moment = Eigen::VectorXd::Zero(6);
	// Eigen::Vector3d sensed_force, sensed_moment;
	Eigen::VectorXd sensor_bias = Eigen::VectorXd::Zero(6);

	const int zero_fs_counter = 3000;
	int zero_fs_buffer = zero_fs_counter + 2000;
	int ol_fc_buffer = 4000;

	state = INITIAL;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_KEY, sensed_force_moment);

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateKinematics();
			if(!simulation)
			{
				redis_client.getEigenMatrixDerived(MASSMATRIX_KEY, robot->_M);
				robot->_M_inv = robot->_M.inverse();
			}
			else
			{
				robot->updateDynamics();
			}

			// ----------- tasks
			N_prec = Eigen::MatrixXd::Identity(dof,dof);

			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//---- orientation controller
		// update current orientation
		robot->rotation(posori_task->_current_orientation, posori_task->_link_name);
		robot->angularVelocity(posori_task->_current_angular_velocity, posori_task->_link_name);

		//---- position controller 
		// update current position and velocity
		robot->position(posori_task->_current_position, posori_task->_link_name, posori_task->_T_link_control.translation());
		robot->linearVelocity(posori_task->_current_linear_velocity, posori_task->_link_name, posori_task->_T_link_control.translation());

		// std::cout << "desired z position : " << posori_task->desired_position(2) << std::endl;
		// std::cout << "z position : " << posori_task->current_position(2) << std::endl;

		if(state == INITIAL)
		{
			state = MOVE_ABOVE_CONTACT;
			std::cout << "Move above contact\n" << std::endl;
		}

		if(state == MOVE_ABOVE_CONTACT)
		{
			posori_task->_desired_position += Eigen::Vector3d(0, 0, -0.00015);
			if(posori_task->_current_position(2) < initial_position(2) - 0.25)
			{
				if(simulation)
				{
					state = GO_TO_CONTACT;
					std::cout << "Go to contact\n" << std::endl;
				}
				else
				{
					state = ZERO_FORCE_SENSOR;
					std::cout << "Zero force sensor\n" << std::endl;
				}
			}
		}

		else if(state == ZERO_FORCE_SENSOR)
		{
			if(zero_fs_buffer > zero_fs_counter)
			{
				zero_fs_buffer--;
			}
			else if(zero_fs_buffer > 0)
			{
				sensor_bias += sensed_force_moment;
				zero_fs_buffer--;
			}
			else
			{
				sensor_bias /= zero_fs_counter;
				sensor_bias.tail(3) << 0,0,0;
				state = GO_TO_CONTACT;
				std::cout << "Go to contact\n" << std::endl;
			}

		}

		else if(state == GO_TO_CONTACT)
		{
			// rotate forces to world frame and unbias, and set to controller
			sensed_force_moment -= sensor_bias;
			if(simulation)
			{
				sensed_force_moment = -sensed_force_moment;
			}
			posori_task->updateSensedForceAndMoment(-sensed_force_moment.head(3), -sensed_force_moment.tail(3));

			if(controller_counter % 25 == 0)
			{
				posori_task->_desired_position += Eigen::Vector3d(0,0,-0.00050);
			}
			if(sensed_force_moment(2) < -5)
			{
				state = OL_FORCE_CONTROL;
				posori_task->setForceAxis(Eigen::Vector3d(0,0,1));
				posori_task->_desired_force = Eigen::Vector3d(0,0,-7);
				std::cout << "Open loop force control\n" << std::endl;
			}
		}

		else if(state == OL_FORCE_CONTROL)
		{
			sensed_force_moment -= sensor_bias;
			if(simulation)
			{
				sensed_force_moment = -sensed_force_moment;
			}
			posori_task->updateSensedForceAndMoment(-sensed_force_moment.head(3), -sensed_force_moment.tail(3));
			
			posori_task->setForceAxis(Eigen::Vector3d(0.0,0.0,1.0));

			Eigen::Vector3d localz = posori_task->_current_orientation.block<3,1>(0,2);

			if(ol_fc_buffer == 0)
			{
				posori_task->_kp_force = 1.0;
				posori_task->_ki_force = 0.7;
				posori_task->_kv_force = 20.0;
				posori_task->_desired_force = 10.0*localz;
				// posori_task->setClosedLoopForceControl();

				// state = CL_FORCE_CONTROL;
				// std::cout << "Closed loop force control\n" << std::endl;
			}
			ol_fc_buffer--;

		}


		else if(state == CL_FORCE_CONTROL)
		{
			sensed_force_moment -= sensor_bias;
			if(simulation)
			{
				sensed_force_moment = -sensed_force_moment;
			}
			posori_task->updateSensedForceAndMoment(-sensed_force_moment.head(3), -sensed_force_moment.tail(3));

			Eigen::Vector3d localz = posori_task->_current_orientation.block<3,1>(0,2);
			// if(controller_counter > 13000)
			// {
				// posori_task->desired_force = Eigen::Vector3d(0.0,0.0,8.0);
			// }

			if(controller_counter % 1000 == 0)
			{
				std::cout << "contoller counter : " << controller_counter << std::endl;
				std::cout << "Rc force : " << posori_task->_Rc_force << std::endl;
				std::cout << "PO force : " << posori_task->_PO_force << std::endl;
				std::cout << std::endl;
			}
		}

		// compute joint torques
		posori_task->computeTorques(posori_task_torques);

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = posori_task_torques + N_prec.transpose()*joint_task_torques;

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		redis_client.setEigenMatrixDerived(EE_DESIRED_FORCE_LOGGED_KEY, posori_task->_desired_force);
		redis_client.setEigenMatrixDerived(EE_SENSED_FORCE_LOGGED_KEY, sensed_force_moment.head(3));
		redis_client.setCommandIs(RC_KEY,std::to_string(posori_task->_Rc_force));

		controller_counter++;

	}

    command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
