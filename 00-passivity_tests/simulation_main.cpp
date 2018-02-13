// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "Sai2Simulation.h"
#include "force_sensor/ForceSensorSim.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <dynamics3d.h>

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

const string world_file = "../resources/00-passivity_tests/world.urdf";
const string robot_file = "../resources/00-passivity_tests/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

// redis keys:
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::timestamp";
const std::string EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force_moment";

const std::string DISTURBANCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::disturbance";

unsigned long long sim_counter = 0;

// force sensor dof
const int ee_sensor_dof = 3;


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
	auto sim = new Simulation::Sai2Simulation(world_file, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	int dof = robot->dof();
	Eigen::VectorXd robot_torques = Eigen::VectorXd::Zero(dof);

	// create force sensor
	Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity();
	sensor_frame.translation() = Eigen::Vector3d(0.0, 0.0, 0.02);
	ForceSensorSim* force_sensor = new ForceSensorSim(robot_name, "link6", sensor_frame, robot);
	Eigen::Vector3d sensed_force, sensed_moment;
	Eigen::VectorXd sensed_forces_moments(6);

	// set initial position to match kuka driver
	sim->setJointPosition(robot_name, 0, 90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1, -30.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 3, 60.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 5, -90.0/180.0*M_PI);

	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.3);

	// Get the world gravity for gravity comp
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	Eigen::VectorXd joint_gravity = Eigen::VectorXd::Zero(dof);

	// create a loop timer 
	double sim_freq = 10000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	std::string dist;
	redis_client.setCommandIs(DISTURBANCE_KEY,"0");
	Eigen::VectorXd disturbance_torque(dof);

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, robot_torques);
		redis_client.getCommandIs(DISTURBANCE_KEY, dist);

		if(dist == "1")
		{
			Eigen::Vector3d disturbance_force = Eigen::Vector3d(0.0,1.0,5.0);
			Eigen::MatrixXd J_dist;
			robot->Jv(J_dist,"link5", Eigen::Vector3d::Zero());
			disturbance_torque = J_dist.transpose()*disturbance_force;
		}
		else
		{
			disturbance_torque.setZero();
		}

		// robot_plus_sensor_torques << robot_torques, ee_sensor_force;
		sim->setJointTorques(robot_name, robot_torques + joint_gravity + disturbance_torque);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		robot->gravityVector(joint_gravity,world_gravity);

		// update force sensor and display
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);
		force_sensor->getMomentLocalFrame(sensed_moment);
		sensed_forces_moments << sensed_force, sensed_moment;

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_KEY, -sensed_forces_moments);
		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		sim_counter++;

		if(sim_counter % 500 == 0)
		{
			// std::cout << "force sensor position : \t" << ee_sensor_position.transpose() << std::endl;
			std::cout << "end effector sensor forces : \t" << sensed_forces_moments.head(3).transpose() << std::endl;
			std::cout << std::endl;
		}

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
