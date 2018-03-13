// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "primitives/RedundantArmMotion.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "../resources/util-fsensor_calibration/iiwa7.urdf";
const std::string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
// - read:
std::string JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
std::string EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";

int main() {
	std::cout << "Loading URDF robot model file: " << robot_file << std::endl;

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
	control_frame.translation() = Vector3d(0.0, 0.0, -0.11);
	auto motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, control_link, control_frame);

	Vector3d initial_position;
	robot->position(initial_position, control_link, control_frame.translation());

	VectorXd motion_primitive_torques = VectorXd::Zero(dof);

	motion_primitive->_posori_task->_kp_pos = 50.0;
	motion_primitive->_posori_task->_kv_pos = 10.0;
	motion_primitive->_posori_task->_kp_ori = 50.0;
	motion_primitive->_posori_task->_kv_ori = 10.0;

	motion_primitive->_joint_task->_kp = 1.0;
	motion_primitive->_joint_task->_kv = 5.0;

	// mass and center of mass quess
	Vector3d world_gravity = Vector3d(0, 0, -9.81);
	double guessed_mass = 0;

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
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		motion_primitive->computeTorques(motion_primitive_torques);
		command_torques = motion_primitive_torques;

		//------ Final torques
		command_torques << 0, 0, 0, 0, 0, 0, 0;
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		// mass guess :
		guessed_mass = (double) (world_gravity.transpose()*sensed_force_moment.head<3>()) / (double) (world_gravity.transpose()*world_gravity);

		if(controller_counter % 500 == 0)
		{
			cout << "sensed force: " << sensed_force_moment.head(3).transpose() << endl;
			cout << "guessed mass : " << guessed_mass << endl;
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
