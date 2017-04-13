// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "simulation/SimulationInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

const string world_file = "resources/01-force_tracking/world.urdf";
const string robot_file = "../robot_models/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

// redis keys:
// - read:
// const std::string JOINT_ANGLES_DES_KEY  = "scl::robot::iiwaBot::sensors::q_des";
const std::string JOINT_TORQUES_COMMANDED_KEY = "scl::robot::iiwaBot::actuators::fgc";
// - write:
const std::string JOINT_ANGLES_KEY  = "scl::robot::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "scl::robot::iiwaBot::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "scl::robot::iiwaBot::timestamp";
// const std::string FGC_ENABLE_KEY  = "scl::robot::iiwaBot::fgc_command_enabled";
// const std::string JOINT_TORQUES_KEY = "scl::robot::iiwaBot::sensors::fgc";

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

	// load simulation world
	auto sim = new Simulation::SimulationInterface(world_file, Simulation::sai2simulation, Simulation::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// set initial position to match kuka driver
	sim->setJointPosition(robot_name, 0, 90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1, 30.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 3, 60.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 5, 90.0/180.0*M_PI);

	// create a loop timer
	double sim_freq = 1000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop


	Eigen::VectorXd robot_torques(7);
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, robot_torques);
		sim->setJointTorques(robot_name, robot_torques);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		
		// write joint kinematics to redis
		redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot->_dq);

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

	}
	return 0;
}
