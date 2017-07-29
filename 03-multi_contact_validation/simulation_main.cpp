// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "simulation/SimulationInterface.h"
#include "simulation/Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <dynamics3d.h>
#include "force_sensor/ForceSensorSim.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

const string world_file = "../resources/03-multi_contact_validation/world.urdf";
const string robot_file = "../../robot_models/kuka_iiwa/03-multi_contact_validation/kuka_iiwa_base_sensor.urdf";
const string robot_name = "Kuka-IIWA";

// redis keys:
// - read:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - write:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
const std::string SIM_TIMESTAMP_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::timestamp";
const std::string VIRTUAL_BASE_POSITION_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::virtual_base::position";
const std::string BASE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::base_force_sensor::force";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::ee_force_sensor::force";

unsigned long long sim_counter = 0;

const int virtual_base_dof = 6;

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
	auto sim = new Simulation::Sai2Simulation(world_file, Simulation::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// create simulated force sensors
	auto base_sensor = new ForceSensorSim(robot_name, "base_link", Eigen::Affine3d::Identity(), robot);
	auto ee_sensor = new ForceSensorSim(robot_name, "link6", Eigen::Affine3d::Identity(), robot);

	// set initial position to match kuka driver
	sim->setJointPosition(robot_name, 0+virtual_base_dof, 90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1+virtual_base_dof, 5/180.0*M_PI);
	sim->setJointPosition(robot_name, 3+virtual_base_dof, 15/180.0*M_PI);
	sim->setJointPosition(robot_name, 5+virtual_base_dof, 15/180.0*M_PI);
	// for(int i=0; i < 7+virtual_base_dof; i++)
	// {
	// 	sim->setJointPosition(robot_name, i, 0);
	// }

	sim->setCollisionRestitution(0);

	// Get the world gravity for gravity comp
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	// auto world_gravity_tmp = sim->_world->m_dynWorld->gravity;
	// Eigen::Vector3d world_gravity = Eigen::Vector3d(world_gravity_tmp[0],world_gravity_tmp[1],world_gravity_tmp[2]);

	// std::cout << "gravity : " << world_gravity.transpose() << std::endl;

	// create a loop timer
	double sim_freq = 10000.0;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd robot_torques(7);
	robot_torques.setZero();
	Eigen::VectorXd robot_plus_base_torques(virtual_base_dof + 7);
	Eigen::VectorXd joint_gravity(virtual_base_dof + 7);

	Eigen::VectorXd virtual_base_torques = Eigen::VectorXd::Zero(virtual_base_dof);
	Eigen::VectorXd virtual_base_position = Eigen::VectorXd::Zero(virtual_base_dof);

	Eigen::VectorXd base_force_and_moment(6);
	Eigen::Vector3d base_force;
	Eigen::Vector3d base_moment;
	Eigen::VectorXd ee_force_and_moment(6);
	Eigen::Vector3d ee_force;
	Eigen::Vector3d ee_moment;
	// base_force_and_moment.setZero();
	// base_moment.setZero();
	// ee_force_and_moment.setZero();
	// ee_moment.setZero();

	Eigen::VectorXd robot_positions(7);
	Eigen::VectorXd robot_velocities(7);

	robot->updateModel();
	robot->gravityVector(joint_gravity,world_gravity);
	joint_gravity.head<virtual_base_dof>() = Eigen::VectorXd::Zero(virtual_base_dof);

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, robot_torques);

		robot_plus_base_torques << virtual_base_torques, robot_torques;
		sim->setJointTorques(robot_name, robot_plus_base_torques + joint_gravity);
		// sim->setJointTorques(robot_name, Eigen::VectorXd::Zero(virtual_base_dof + 7));

		// update simulation by 1ms
		// sim->integrate(1.0/sim_freq);
		// sim->integrate(0.001/sim_freq*1000.0);
		sim->integrate(0.001);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot_positions = robot->_q.segment<7>(virtual_base_dof);
		robot_velocities = robot->_dq.segment<7>(virtual_base_dof);

		virtual_base_position = robot->_q.head<virtual_base_dof>();

		robot->updateModel();
		robot->gravityVector(joint_gravity,world_gravity);
		joint_gravity.head<virtual_base_dof>() = Eigen::VectorXd::Zero(virtual_base_dof);

		base_sensor->update(sim);
		ee_sensor->update(sim);
		base_sensor->getForce(base_force);
		base_sensor->getMoment(base_moment);
		ee_sensor->getForce(ee_force);
		ee_sensor->getMoment(ee_moment);
		base_force_and_moment << base_force, base_moment;
		ee_force_and_moment << ee_force, ee_moment;


		// write joint kinematics to redis
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, robot_positions);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot_velocities);
		redis_client.setEigenMatrixDerived(VIRTUAL_BASE_POSITION_KEY, virtual_base_position);

		redis_client.setEigenMatrixDerived(BASE_FORCE_SENSOR_FORCE_KEY, base_force_and_moment);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, ee_force_and_moment);

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		sim_counter++;

		if(sim_counter % 500 == 0)
		{
			std::cout << "base force : " << base_force.transpose() << std::endl;
			std::cout << "base moment : " << base_moment.transpose() << std::endl;
			std::cout << "ee force : " << ee_force.transpose() << std::endl;
			std::cout << "ee moment : " << ee_moment.transpose() << std::endl;
			std::cout << "\n\n" << std::endl;
		}

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
