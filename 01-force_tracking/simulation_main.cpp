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
const string robot_file = "../robot_models/kuka_iiwa/kuka_iiwa_force_sensor.urdf";
const string robot_name = "Kuka-IIWA";

// redis keys:
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::timestamp";
const std::string FORCE_SENSOR_POSITION_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::position";
const std::string FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::force_sensor::force";

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
	double sim_freq = 10000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop


	Eigen::VectorXd robot_torques(7);
	double sensor_force = 0.0;
	Eigen::VectorXd robot_plus_sensor_torques(8);

	double sensor_position = 0.0;
	double sensor_velocity = 0.0;
	double sensor_stiffness = 3000;
	double sensor_damping = 400;
	Eigen::VectorXd robot_positions(7);
	Eigen::VectorXd robot_velocities(7);
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerivedString(JOINT_TORQUES_COMMANDED_KEY, robot_torques);

		robot_plus_sensor_torques << robot_torques, sensor_force;
		sim->setJointTorques(robot_name, robot_plus_sensor_torques);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// TODO : compute sensor force from sensor position (when to do it?? before or after integrate)
		sensor_force = -sensor_stiffness*sensor_position - sensor_damping*sensor_velocity;

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot_positions = robot->_q.head<7>();
		sensor_position = robot->_q(7);
		robot_velocities = robot->_dq.head<7>();
		sensor_velocity = robot->_dq(7);
		robot->updateModel();

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot_positions);
		redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot_velocities);
		redis_client.setCommandIs(FORCE_SENSOR_POSITION_KEY, std::to_string(sensor_position));
		redis_client.setCommandIs(FORCE_SENSOR_FORCE_KEY, std::to_string(-sensor_force));

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

	}
	return 0;
}
