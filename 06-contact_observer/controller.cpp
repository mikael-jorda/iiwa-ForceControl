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
#include "momentum_observer/MomentumObserver.h"

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

#define INITIAL                0
#define GC_STOP                1

using namespace std;
int state;

const string world_file = "../resources/06-contact_observer/world.urdf";
const string robot_file = "../resources/06-contact_observer/kuka_iiwa.urdf";
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

const std::string EE_POS_KEY = "sai2::iiwaForceControl::iiwaBot::data_log::position";
const std::string EE_DPOS_KEY = "sai2::iiwaForceControl::iiwaBot::data_log::_desired_position";

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
	Eigen::VectorXd anti_grav_comp(dof), gravity_vector(dof);
	joint_task_desired_position.setZero();
	joint_task_torques.setZero();
	anti_grav_comp.setZero();
	gravity_vector.setZero();

	double joint_kv = 5.0;

	// position and orientation task
	const string control_link = "link6";
	Eigen::Affine3d control_frame = Eigen::Affine3d::Identity();
	control_frame.translation() = Eigen::Vector3d(0.0, 0.0, 0.12);
	Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity();
	sensor_frame.translation() = Eigen::Vector3d(0.0, 0.0, 0.02);

	auto posori_task = new PosOriTask(robot, "link6", control_frame, sensor_frame);

	posori_task->_kp_ori = 600.0;
	// posori_task->_kp_ori = 0.0;
	posori_task->_kv_ori = 50.0;
	posori_task->_kp_pos = 150.0;
	// posori_task->_kp_pos = 0.0;
	posori_task->_kv_pos = 22.0;

	Eigen::VectorXd posori_task_torques(dof);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd sensed_force_moment = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd sensor_bias = Eigen::VectorXd::Zero(6);

	// prepare momentum observer
	MomentumObserver* mobs = new MomentumObserver(robot, 1.0/control_freq, true);
	Eigen::VectorXd r(dof);
	r.setZero();


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

		robot->gravityVector(gravity_vector);

		// update momentum observer
		mobs->update(r, command_torques);

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

		if(state == INITIAL)
		{
			if(controller_counter > 1000)
			{
				posori_task->_desired_position += Eigen::Vector3d(0.1,0.0,0.0);
				state = GC_STOP;
				cout << "stop nulspace gavity compensation\n" << endl;
			}
		}

		else if(state == GC_STOP)
		{
			anti_grav_comp = N_prec.transpose() * gravity_vector;
			// cout << gravity_vector.transpose() << endl;
		}

		// compute joint torques
		posori_task->computeTorques(posori_task_torques);
		posori_task_torques -= posori_task->_projected_jacobian.transpose() * posori_task->_Jbar.transpose() * r;

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = posori_task_torques + N_prec.transpose()*joint_task_torques - anti_grav_comp;

		// command_torques -= r;

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// redis_client.setEigenMatrixDerived(EE_DESIRED_FORCE_LOGGED_KEY, posori_task->_desired_force);
		// redis_client.setEigenMatrixDerived(EE_SENSED_FORCE_LOGGED_KEY, sensed_force_moment.head(3));
		// redis_client.setCommandIs(RC_KEY,std::to_string(posori_task->_Rc_force));

		redis_client.setEigenMatrixDerived(EE_POS_KEY, posori_task->_current_position);
		redis_client.setEigenMatrixDerived(EE_DPOS_KEY, posori_task->_desired_position);

		if(controller_counter % 500 == 0)
		{
			cout << "momentum observer : " << r.transpose() << endl; 
		}

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
