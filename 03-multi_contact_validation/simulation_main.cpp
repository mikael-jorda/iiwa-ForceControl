// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "simulation/SimulationInterface.h"
#include "simulation/Sai2Simulation.h"
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
const std::string BASE_FORCE_SENSOR_POSITION_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::base_force_sensor::position";
const std::string BASE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::base_force_sensor::force";
const std::string EE_FORCE_SENSOR_POSITION_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::ee_force_sensor::position";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::ee_force_sensor::force";

unsigned long long sim_counter = 0;

// force sensor dof
const int base_sensor_dof = 3;
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
	auto sim = new Simulation::Sai2Simulation(world_file, Simulation::urdf, false);

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// set initial position to match kuka driver
	sim->setJointPosition(robot_name, 0+base_sensor_dof, 90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1+base_sensor_dof, -30.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 3+base_sensor_dof, 60.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 5+base_sensor_dof, -90.0/180.0*M_PI);

	sim->setCollisionRestitution(0);

	// Get the world gravity for gravity comp
	Eigen::Vector3d world_gravity = sim->_world->getGravity().eigen();
	// auto world_gravity_tmp = sim->_world->m_dynWorld->gravity;
	// Eigen::Vector3d world_gravity = Eigen::Vector3d(world_gravity_tmp[0],world_gravity_tmp[1],world_gravity_tmp[2]);

	std::cout << "gravity : " << world_gravity.transpose() << std::endl;

	// create a loop timer
	double sim_freq = 5000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd robot_torques(7);
	Eigen::VectorXd robot_plus_sensor_torques(base_sensor_dof + 7 + ee_sensor_dof);
	Eigen::VectorXd joint_gravity(base_sensor_dof + 7 + ee_sensor_dof);

	Eigen::VectorXd base_sensor_force(base_sensor_dof);
	Eigen::Vector3d base_sensor_position(base_sensor_dof);
	Eigen::Vector3d base_sensor_velocity(base_sensor_dof);
	Eigen::VectorXd ee_sensor_force(ee_sensor_dof);
	Eigen::Vector3d ee_sensor_position(ee_sensor_dof);
	Eigen::Vector3d ee_sensor_velocity(ee_sensor_dof);

	Eigen::MatrixXd base_sensor_stiffness = 100000*Eigen::MatrixXd::Identity(base_sensor_dof,base_sensor_dof);
	Eigen::MatrixXd base_sensor_damping = 800*Eigen::MatrixXd::Identity(base_sensor_dof,base_sensor_dof);
	Eigen::MatrixXd ee_sensor_stiffness = 100000*Eigen::MatrixXd::Identity(ee_sensor_dof,ee_sensor_dof);
	Eigen::MatrixXd ee_sensor_damping = 800*Eigen::MatrixXd::Identity(ee_sensor_dof,ee_sensor_dof);

	Eigen::VectorXd robot_positions(7);
	Eigen::VectorXd robot_velocities(7);


	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read torques from Redis
		redis_client.getEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, robot_torques);

		robot_plus_sensor_torques << base_sensor_force, robot_torques, ee_sensor_force;
		sim->setJointTorques(robot_name, robot_plus_sensor_torques + joint_gravity);

		// update simulation by 1ms
		sim->integrate(1/sim_freq);

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot_positions = robot->_q.segment<7>(base_sensor_dof);
		robot_velocities = robot->_dq.segment<7>(base_sensor_dof);

		base_sensor_position = robot->_q.head<base_sensor_dof>();
		base_sensor_velocity = robot->_dq.head<base_sensor_dof>();
		ee_sensor_position = robot->_q.tail<ee_sensor_dof>();
		ee_sensor_velocity = robot->_dq.tail<ee_sensor_dof>();

		robot->updateModel();
		robot->gravityVector(joint_gravity,world_gravity);
		joint_gravity.head<base_sensor_dof>() << 0,0,0;
		joint_gravity.tail<ee_sensor_dof>() << 0,0,0;


		// compute sensor force from relaxed sensor position
		base_sensor_force = -base_sensor_stiffness*base_sensor_position - base_sensor_damping*base_sensor_velocity;
		ee_sensor_force = -ee_sensor_stiffness*ee_sensor_position - ee_sensor_damping*ee_sensor_velocity;

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerived(JOINT_ANGLES_KEY, robot_positions);
		redis_client.setEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot_velocities);
		redis_client.setEigenMatrixDerived(BASE_FORCE_SENSOR_FORCE_KEY, base_sensor_force);
		redis_client.setEigenMatrixDerived(BASE_FORCE_SENSOR_POSITION_KEY, base_sensor_position);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, ee_sensor_force);
		redis_client.setEigenMatrixDerived(EE_FORCE_SENSOR_POSITION_KEY, ee_sensor_position);

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		sim_counter++;

		if(sim_counter % 500 == 0)
		{
			std::cout << "base sensor forces : \t" << base_sensor_force.transpose() << std::endl;
			std::cout << "end effector sensor forces : \t" << ee_sensor_force.transpose() << std::endl;
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
