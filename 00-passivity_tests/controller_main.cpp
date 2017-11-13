// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include "tasks/OrientationTask.h"
// #include "tasks/HybridPositionTask.h"
#include "tasks/TestTask.h"

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

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;

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
	// auto robot = new Model::ModelInterface(robot_file, Model::rbdl_kuka, Model::urdf, false);
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

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

	// Orientation task
	OrientationTask ori_task = OrientationTask(dof);
	ori_task.link_name = "link6";
	ori_task.setKp(600.0);
	ori_task.setKv(50.0);

	Eigen::VectorXd ori_task_torques(dof);
	Eigen::Matrix3d initial_orientation;
	robot->rotation(initial_orientation, ori_task.link_name);
	ori_task.desired_orientation = initial_orientation;
	ori_task.desired_orientation = initial_orientation;

	// operational space position task
	TestTask pos_task =	TestTask(dof);
	pos_task.link_name = "link5";
	pos_task.pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0);
	pos_task.setKp(150.0);
	pos_task.setKv(15.0);
	pos_task.useVelocitySaturation(true, Eigen::Vector3d(0.2, 0.2, 0.2));

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

	Eigen::VectorXd sensed_force_moment = Eigen::VectorXd::Zero(6);
	Eigen::Vector3d sensed_force, sensed_moment;
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
			robot->updateModel();

			/////////////////////////// update jacobians, mass matrices and nullspaces
			N_prec = Eigen::MatrixXd::Identity(dof,dof);

			// orientation controller
			robot->Jw(ori_task.jacobian, ori_task.link_name);
			ori_task.projected_jacobian = ori_task.jacobian * N_prec;
			robot->operationalSpaceMatrices(ori_task.Lambda, ori_task.Jbar, ori_task.N,
									ori_task.projected_jacobian, N_prec);
			N_prec = ori_task.N;

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

		// compute torques
		ori_task.computeTorques(ori_task_torques);


		//---- position controller 
		// update current position and sensor force
		robot->position(pos_task.current_position, pos_task.link_name, pos_task.pos_in_link);
		pos_task.current_velocity = pos_task.projected_jacobian * robot->_dq;

		// std::cout << "desired z position : " << pos_task.desired_position(2) << std::endl;
		// std::cout << "z position : " << pos_task.current_position(2) << std::endl;

		if(state == INITIAL)
		{
			pos_task.desired_position += Eigen::Vector3d(0, 0, -0.25);
			state = MOVE_ABOVE_CONTACT;
			std::cout << "Move above contact\n" << std::endl;
		}

		if(state == MOVE_ABOVE_CONTACT)
		{
			if(pos_task.current_position(2) < 1.01*pos_task.desired_position(2))
			{
				state = ZERO_FORCE_SENSOR;
				std::cout << "Zero force sensor\n" << std::endl;
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
				state = GO_TO_CONTACT;
				std::cout << "Go to contact\n" << std::endl;
			}

		}

		else if(state == GO_TO_CONTACT)
		{
			// rotate forces to world frame and unbias, and set to controller
			if(simulation)
			{
				sensed_force_moment = sensed_force_moment - sensor_bias;
			}
			else
			{
				Eigen::MatrixXd R6d_sensor = Eigen::MatrixXd::Zero(6,6);
				R6d_sensor.block<3,3>(0,0) = ori_task.current_orientation;
				R6d_sensor.block<3,3>(3,3) = ori_task.current_orientation;
				sensed_force_moment = R6d_sensor * (sensed_force_moment - sensor_bias);
			}
			if(controller_counter % 25 == 0)
			{
				pos_task.desired_position += Eigen::Vector3d(0,0,-0.00050);
			}
			if(sensed_force_moment(2) < -5)
			{
				state = OL_FORCE_CONTROL;
				pos_task.setForceAxis(Eigen::Vector3d(0,0,1));
				pos_task.desired_force = Eigen::Vector3d(0,0,-7);
				std::cout << "Open loop force control\n" << std::endl;
			}
		}

		else if(state == OL_FORCE_CONTROL)
		{
			// rotate forces to world frame and unbias, and set to controller
			if(simulation)
			{
				sensed_force_moment = sensed_force_moment - sensor_bias;
			}
			else
			{
				Eigen::MatrixXd R6d_sensor = Eigen::MatrixXd::Zero(6,6);
				R6d_sensor.block<3,3>(0,0) = ori_task.current_orientation;
				R6d_sensor.block<3,3>(3,3) = ori_task.current_orientation;
				sensed_force_moment = R6d_sensor * (sensed_force_moment - sensor_bias);
			}				
			if(ol_fc_buffer == 0)
			{
				pos_task.setKpf(1.3);
				pos_task.setKif(1.0);
				pos_task.desired_force = Eigen::Vector3d(0,0,-5);
				pos_task.setClosedLoopForceControl(control_freq);
				pos_task.enablePassivity();
				state = CL_FORCE_CONTROL;
				std::cout << "Closed loop force control\n" << std::endl;
			}
			ol_fc_buffer--;
			// if(controller_counter % 500 == 0)
			// {
				// std::cout << "force related forces : " << pos_task.force_related_forces.transpose() << std::endl; 
				// std::cout << "position related forces : " << pos_task.position_related_forces.transpose() << std::endl; 
				// std::cout << "position task torques : " << pos_task_torques.transpose() << std::endl; 
			// }
		}

		else if(state == CL_FORCE_CONTROL)
		{
			// rotate forces to world frame and unbias, and set to controller
			if(simulation)
			{
				sensed_force_moment = sensed_force_moment - sensor_bias;
			}
			else
			{
				Eigen::MatrixXd R6d_sensor = Eigen::MatrixXd::Zero(6,6);
				R6d_sensor.block<3,3>(0,0) = ori_task.current_orientation;
				R6d_sensor.block<3,3>(3,3) = ori_task.current_orientation;
				sensed_force_moment = R6d_sensor * (sensed_force_moment - sensor_bias);
			}
			pos_task.sensed_force = sensed_force_moment.head(3);

			if(controller_counter % 1000 == 0)
			{
				std::cout << "Rc : " << pos_task.Rc_ << std::endl;
			}
		}

		if(controller_counter % 1000 == 0)
		{
			// std::cout << "end effector orientation : \n" << ori_task.current_orientation << std::endl;
			// std::cout << "sensor force in sensor frame : \n" << ee_sensed_force.transpose() << std::endl;
			std::cout << "\nsensor force in world frame : \n" << sensed_force_moment.head(3).transpose() << std::endl;
			std::cout << "power input " << pos_task.current_power_input_ << std::endl;
			std::cout << "power output " << pos_task.current_power_output_ << std::endl;
			std::cout << "vc " << pos_task.force_feedback_control_signal.transpose() << std::endl;
		}

		// compute joint torques
		pos_task.computeTorques(pos_task_torques);

		//----- Joint nullspace damping
		joint_task_torques = robot->_M*( - joint_kv*robot->_dq);

		//------ Final torques
		command_torques = pos_task_torques + ori_task_torques + N_prec.transpose()*joint_task_torques;

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);


		redis_client.setEigenMatrixDerived(EE_DESIRED_FORCE_LOGGED_KEY, pos_task.desired_force);
		redis_client.setEigenMatrixDerived(EE_SENSED_FORCE_LOGGED_KEY, sensed_force_moment.head(3));
		redis_client.setCommandIs(RC_KEY,std::to_string(pos_task.Rc_));

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
