// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "primitives/RedundantArmMotion.h"
#include "primitives/SurfaceSurfaceAlignment.h"

#include <iostream>
#include <string>
#include <tinyxml2.h>

// #include "chai3d.h"

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;
// using namespace chai3d;

const string world_file = "../resources/08-surface_alignment/world.urdf";
const string robot_file = "../resources/08-surface_alignment/iiwa7_surface_alignment.urdf";
const std::string robot_name = "Kuka-IIWA";


unsigned long long controller_counter = 0;

// const bool simulation = true;
const bool simulation = false;

// sensor calibration
const string path_to_calibration_file = "../../08-surface_alignment/cube_calibration.xml";
const string path_to_bias_file = "../../08-surface_alignment/sensor_bias.xml";
void readBias(VectorXd& sensor_bias);
void readObjectCalibration(Vector3d& com, double& mass);

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY;
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string EE_FORCE_SENSOR_KEY;
// const std::string MASSMATRIX_KEY = "sai2::KUKA_IIWA::sensors::massmatrix";

const std::string EE_SENSED_FORCE_LOGGER_KEY = "sai2::iiwaForceControl::logger::force_moment";
const std::string EE_POSORI_CONTROL_FORCE_LOGGER_KEY = "sai2::iiwaForceControl::logger::ori_task_force";

#define   GO_TO_CONTACT       0
#define   SURFACE_ALIGNMENT   1
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
	const string sensor_link = "link7";
	VectorXd sensed_force_moment = VectorXd::Zero(6);

	Vector3d world_gravity = Vector3d(0, 0, -9.81);
	Vector3d sensor_gravity = Vector3d::Zero();
	VectorXd sensor_bias = VectorXd::Zero(6);
	Vector3d tool_com;
	double tool_mass;
	Matrix3d sensor_rotation = Matrix3d::Identity();
	readBias(sensor_bias);
	readObjectCalibration(tool_com, tool_mass);

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
	control_frame.translation() = Vector3d(0.0, 0.0, 0.16);
	auto motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, control_link, control_frame);
	// cout << "task force mot : " << motion_primitive->_posori_task->_task_force.transpose() << endl;

	Vector3d initial_position;
	robot->position(initial_position, control_link, control_frame.translation());

	VectorXd motion_primitive_torques = VectorXd::Zero(dof);

	motion_primitive->_posori_task->_kp_pos = 50.0;
	motion_primitive->_posori_task->_kv_pos = 10.0;
	motion_primitive->_posori_task->_kp_ori = 50.0;
	motion_primitive->_posori_task->_kv_ori = 10.0;

	motion_primitive->_joint_task->_kp = 1.0;
	motion_primitive->_joint_task->_kv = 5.0;

	// surface surface primitive
	// string control_link = "link7";
	Affine3d sensor_frame = Affine3d::Identity();
	sensor_frame.translation() = Vector3d(0.0, 0.0, 0.04);
	auto surface_primitive = new Sai2Primitives::SurfaceSurfaceAlignment(robot, control_link, control_frame);
	// cout << "kp force surf : " << surface_primitive->_posori_task->_kp_force << endl;
	// cout << "task force surf : " << surface_primitive->_posori_task->_task_force.transpose() << endl;


	// surface_primitive->updatePrimitiveModel();

	VectorXd surface_primitive_torques = VectorXd::Zero(dof);
	// surface_primitive->computeTorques(surface_primitive_torques);

	int stabilization_counter = 0;

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	int state;
	state = GO_TO_CONTACT;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_KEY, sensed_force_moment);
		robot->rotation(sensor_rotation, sensor_link);
		sensor_gravity = sensor_rotation.transpose() * world_gravity;
		
		if(!simulation)
		{
			sensed_force_moment -= sensor_bias;
			sensed_force_moment.head(3) += tool_mass * sensor_gravity;
			sensed_force_moment.tail(3) += tool_com.cross(tool_mass * sensor_gravity);
		}

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

		if(state == GO_TO_CONTACT)
		{
			motion_primitive->_desired_position(2) -= 0.00005;
			motion_primitive->computeTorques(motion_primitive_torques);
			
			command_torques = motion_primitive_torques - coriolis;

			if(sensed_force_moment(2) > 5)
			{
				state = STABILIZE;
				cout << "stabilize" << endl;
			}
		}

		else if(state == STABILIZE)
		{
			surface_primitive->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));
			stabilization_counter--;

			motion_primitive->computeTorques(motion_primitive_torques);
			command_torques = motion_primitive_torques - coriolis;

			if(stabilization_counter <= 0)
			{
				robot->position(surface_primitive->_desired_position, control_link, control_frame.translation());
				surface_primitive->_posori_task->_kp_pos = 25.0;
				surface_primitive->_posori_task->_kv_pos = 14.0;
				state = SURFACE_ALIGNMENT;
				cout << "surface alignment" << endl;
			}
		}

		else if(state == SURFACE_ALIGNMENT)
		{
			surface_primitive->_posori_task->_kp_moment = 2.5;
			surface_primitive->_posori_task->_ki_moment = 1.7;
			surface_primitive->_posori_task->_kv_moment = 8.0;
			// surface_primitive->_posori_task->_kv_pos = 14.0;
			surface_primitive->_joint_task->_kp = 5.0;
			surface_primitive->updateSensedForceAndMoment(sensed_force_moment.head(3), sensed_force_moment.tail(3));
			surface_primitive->computeTorques(surface_primitive_torques);
			command_torques = surface_primitive_torques;
		}

		//------ Final torques
		// command_torques.setZero();
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
			// cout << "haptic position : " << h_position.eigen().transpose() << endl;
			// cout << "haptic force : " << h_force_feedback.eigen().transpose() << endl;
			// cout << "sensed force moment : " << sensed_force_moment.transpose() << endl;
			// cout << "surface_primitive kp : " << surface_primitive->_posori_task->_kp_moment << endl;
			// cout << endl;
		}

		// logger quantities
		redis_client.setEigenMatrixDerived(EE_SENSED_FORCE_LOGGER_KEY, sensed_force_moment);
		redis_client.setEigenMatrixDerived(EE_POSORI_CONTROL_FORCE_LOGGER_KEY, surface_primitive->_posori_task->_task_force);

		controller_counter++;


	}

    // command_torques << 0,0,0,0,0,0,0;
    redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

    double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Loop run time  : " << end_time << " seconds\n";
    std::cout << "Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}

void readBias(VectorXd& sensor_bias)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(path_to_bias_file.c_str());
	if (!doc.Error())
	{
		cout << "Loading bias file file ["+path_to_bias_file+"]." << endl;
		try 
		{

			std::stringstream bias( doc.FirstChildElement("force_bias")->
				Attribute("value"));
			bias >> sensor_bias(0);
			bias >> sensor_bias(1);
			bias >> sensor_bias(2);
			bias >> sensor_bias(3);
			bias >> sensor_bias(4);
			bias >> sensor_bias(5);
			std::stringstream ss; ss << sensor_bias.transpose();
			cout << "Sensor bias : "+ss.str() << endl;
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			cout << "WARNING : Failed to parse bias file." << endl;
		}
	} 
	else 
	{
		cout << "WARNING : Could no load bias file ["+path_to_bias_file+"]" << endl;
		doc.PrintError();
	}
}

void readObjectCalibration(Vector3d& com, double& mass)
{
	tinyxml2::XMLDocument doc;
	doc.LoadFile(path_to_calibration_file.c_str());
	if (!doc.Error())
	{
		cout << "Loading bias file file ["+path_to_calibration_file+"]." << endl;
		try 
		{
			std::string parsed_mass = doc.FirstChildElement("tool")->
			FirstChildElement("inertial")->
			FirstChildElement("mass")->
			Attribute("value");
			mass = std::stod(parsed_mass);
			cout << "Tool mass: " << mass << endl;

			std::stringstream parsed_com( doc.FirstChildElement("tool")->
				FirstChildElement("inertial")->
				FirstChildElement("origin")->
				Attribute("xyz"));
			parsed_com >> com(0);
			parsed_com >> com(1);
			parsed_com >> com(2);
			cout << "Tool CoM : " << com.transpose() << endl;;
		}
		catch( const std::exception& e ) // reference to the base of a polymorphic object
		{ 
			std::cout << e.what(); // information from length_error printed
			cout << "WARNING : Failed to parse bias file." << endl;
		}
	} 
	else 
	{
		cout << "WARNING : Could no load bias file ["+path_to_calibration_file+"]" << endl;
		doc.PrintError();
	}
}
