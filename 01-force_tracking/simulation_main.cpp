// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "model/ModelInterface.h"
#include "model/RBDLModel.h"
#include "simulation/SimulationInterface.h"
#include "simulation/Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;

const string world_file = "../resources/01-force_tracking/world.urdf";
const string robot_file = "../../robot_models/kuka_iiwa/01-force_tracking/kuka_iiwa_force_sensor.urdf";
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

unsigned long long sim_counter = 0;

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
	auto robotrbdl = new Model::RBDLModel(robot_file, Model::urdf, false);

	// set initial position to match kuka driver
	sim->setJointPosition(robot_name, 0, 90.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 1, -30.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 3, 60.0/180.0*M_PI);
	sim->setJointPosition(robot_name, 5, -90.0/180.0*M_PI);

	// create a loop timer
	double sim_freq = 5000;  // set the simulation frequency. Ideally 10kHz
	LoopTimer timer;
	timer.setLoopFrequency(sim_freq);   // 10 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	// Subcycling parameters : how many relaxation step for the force sensor per loop
	int subcycling_steps = 100;
	double subcycle_dt = 1/sim_freq/subcycling_steps;
	
	std::vector<Eigen::Vector3d> contact_points;
	std::vector<Eigen::Vector3d> contact_forces;
	sim->setCollisionRestitution(0);

	Eigen::VectorXd robot_torques(7);
	double sensor_force = 0.0;
	Eigen::VectorXd robot_plus_sensor_torques(8);

	double sensor_position = 0.0;
	double sensor_velocity = 0.0;
	double sensor_stiffness = 10000000;
	double sensor_damping = 0.9*2*sqrt(sensor_stiffness);
	double ee_mass;
	Eigen::Vector3d ee_com;

	robotrbdl->getLinkMass(ee_mass, ee_com, "end_effector");

	double x0, x1, x2;

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

		// update kinematic models
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot_positions = robot->_q.head<7>();
		robot_velocities = robot->_dq.head<7>();

		sensor_position = robot->_q(7);
		sensor_velocity = robot->_dq(7);

		// relax the force sensor
		sim->getContactList(contact_points, contact_forces, robot_name, "end_effector");
		if(!contact_points.empty())
		{
			std::cout << "subcycling" << std::endl; 
			int n = contact_points.size();

			// set the local parameters
			double m = ee_mass;
			double k = sensor_stiffness;
			double b = sensor_damping;
			double dt = subcycle_dt;

			double F = 0;
			for(int k=0; k<n; k++)
			{
				F += contact_forces[k](2);
			}

			x0 = sensor_position;
			x1 = sensor_position + sensor_velocity*subcycle_dt;

			std::cout << "initial position and velocity : " << sensor_position << " " << sensor_velocity << std::endl;
			std::cout << "initial sensor force : " << -sensor_stiffness*sensor_position - sensor_damping*sensor_velocity << std::endl;

			for(int i=0; i<subcycling_steps; i++)
			{
				x2 = (F*dt*dt + x0*(b*dt-m-k*dt*dt) + x1*(2*m-b*dt))/m;

				x0 = x1;
				x1 = x2;
			}

			sensor_position = x2;
			sensor_velocity = (x2-x1)/dt;

			std::cout << "final position and velocity : " << sensor_position << " " << sensor_velocity << std::endl;
			std::cout << "final sensor force : " << -sensor_stiffness*sensor_position - sensor_damping*sensor_velocity << "\n" << std::endl;

		}
		
		robot->_q(7) = sensor_position;
		robot->_dq(7) = sensor_velocity;

		sim->setJointPosition(robot_name, 7, sensor_position);
		sim->setJointVelocity(robot_name, 7, sensor_velocity);

		robot->updateModel();

		// compute sensor force from relaxed sensor position
		sensor_force = -sensor_stiffness*sensor_position - sensor_damping*sensor_velocity;

		// write joint kinematics to redis
		redis_client.setEigenMatrixDerivedString(JOINT_ANGLES_KEY, robot_positions);
		redis_client.setEigenMatrixDerivedString(JOINT_VELOCITIES_KEY, robot_velocities);
		redis_client.setCommandIs(FORCE_SENSOR_POSITION_KEY, std::to_string(sensor_position));
		redis_client.setCommandIs(FORCE_SENSOR_FORCE_KEY, std::to_string(-sensor_force));

		redis_client.setCommandIs(SIM_TIMESTAMP_KEY,std::to_string(timer.elapsedTime()));

		sim_counter++;

	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
