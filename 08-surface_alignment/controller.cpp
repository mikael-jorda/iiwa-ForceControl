// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "primitives/RedundantArmMotion.h"
#include "primitives/SurfaceSurfaceAlignment.h"

#include <iostream>
#include <string>

// #include "chai3d.h"

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;
// using namespace chai3d;

const string world_file = "../resources/07-haptic_demo/world.urdf";
const string robot_file = "../resources/07-haptic_demo/iiwa7_haptic_demo.urdf";
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
// const std::string MASSMATRIX_KEY = "sai2::KUKA_IIWA::sensors::massmatrix";

#define   GO_TO_CONTACT       0
#define   SURFACE_ALIGNMENT   1
#define   ZERO_SENSOR         2
#define   STABILIZE           3

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

	// force sensor
	VectorXd sensed_force_moment = VectorXd::Zero(6);
	VectorXd sensor_bias = VectorXd::Zero(6);

	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	VectorXd coriolis = VectorXd::Zero(dof);

	// motion primitive
	string control_link = "link7";
	Affine3d control_frame = Affine3d::Identity();
	control_frame.translation() = Vector3d(0.0, 0.0, 0.20);
	auto motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, control_link, control_frame);

	Vector3d initial_position;
	robot->position(initial_position, control_link, control_frame.translation());

	VectorXd motion_primitive_torques = VectorXd::Zero(dof);

	motion_primitive->_posori_task->_kp_pos = 50.0;
	motion_primitive->_posori_task->_kv_pos = 10.0;
	motion_primitive->_posori_task->_kp_ori = 50.0;
	motion_primitive->_posori_task->_kv_ori = 10.0;

	motion_primitive->_joint_task->_kp = 10.0;
	motion_primitive->_joint_task->_kv = 5.0;

	// surface surface primitive
	// string control_link = "link7";
	Affine3d sensor_frame = Affine3d::Identity();
	sensor_frame.translation() = Vector3d(0.0, 0.0, 0.20);
	auto surface_primitive = new Sai2Primitives::SurfaceSurfaceAlignment(robot, control_link, control_frame);

	VectorXd surface_primitive_torques = VectorXd::Zero(dof);

	int stabilization_counter = 500;

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	int state;
	if(simulation)
	{
		state = GO_TO_CONTACT;
	}
	else
	{
		state = ZERO_SENSOR;
	}

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

			// robot->coriolisForce(coriolis);

			// ----------- tasks
			motion_primitive->updatePrimitiveModel();
			surface_primitive->updatePrimitiveModel();
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		if(state == ZERO_SENSOR)
		{
			sensor_bias += sensed_force_moment;
			if(controller_counter == 3000)
			{
				sensor_bias /= 3000.0;
				std::cout << "sensor bias : " << sensor_bias.transpose() << endl;
				state = GO_TO_CONTACT;
			}
		}

		else if(state == GO_TO_CONTACT)
		{
			motion_primitive->_desired_position(2) -= 0.00005;
			motion_primitive->computeTorques(motion_primitive_torques);
			
			command_torques = motion_primitive_torques - coriolis;

			if(sensed_force_moment(2) < -5)
			{
				state = STABILIZE;
				cout << "stabilize" << endl;
			}
		}

		else if(state == STABILIZE)
		{
			surface_primitive->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));
			stabilization_counter--;
			if(stabilization_counter == 0)
			{
				state = SURFACE_ALIGNMENT;
				cout << "surface alignment" << endl;
			}
		}

		else if(state == SURFACE_ALIGNMENT)
		{
			surface_primitive->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));
			surface_primitive->computeTorques(surface_primitive_torques);
			command_torques = surface_primitive_torques;
		}

		//------ Final torques

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
			// cout << "haptic position : " << h_position.eigen().transpose() << endl;
			// cout << "haptic force : " << h_force_feedback.eigen().transpose() << endl;
			cout << "sensed force moment : " << sensed_force_moment.transpose() << endl;
			cout << endl;
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
