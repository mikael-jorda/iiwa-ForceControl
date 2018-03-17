// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "tasks/JointTask.h"

#include <iostream>
#include <fstream>
#include <string>
#include <tinyxml2.h>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

#define   MOVE       0
#define   MEASURE    1
#define   END        2

const string robot_file = "../resources/util-fsensor_calibration/iiwa7.urdf";
const std::string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

int simulation = true;
// int simulation = false;

// write xml file
const string path_to_calibration_file = "../../util-fsensor_calibration/calibration_files/object_calibration.xml";
const string path_to_bias_file = "../../util-fsensor_calibration/calibration_files/sensor_bias.xml";
const string tool_name = "test";
void writeXml(const Vector3d object_com, const double object_mass);

// read sensor bias
void readBias(VectorXd& sensor_bias);

// redis keys:
// - write:
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::KUKA_IIWA::actuators::fgc";
// - read:
std::string JOINT_ANGLES_KEY  = "sai2::KUKA_IIWA::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::KUKA_IIWA::sensors::dq";
std::string EE_FORCE_SENSOR_KEY = "sai2::optoforceSensor::6Dsensor::force";

int main() {
	std::cout << "Loading URDF robot model file: " << robot_file << std::endl;

	if(simulation)
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
	}

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

	// read the sensor bias
	VectorXd sensor_bias = VectorXd::Zero(6);
	readBias(sensor_bias);

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof,dof);

	VectorXd coriolis = VectorXd::Zero(dof);

	// controller
	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

	VectorXd q_init = robot->_q;

	VectorXd joint_task_torques(dof);

	joint_task->_kp = 50.0;
	joint_task->_kv = 15.0;

	// bias vector
	VectorXd bias = VectorXd::Zero(6);

	VectorXd initial_position = VectorXd::Zero(dof);
	initial_position << 90.0, -30, 0, 60, 0, -90, 0;
	initial_position *= M_PI/180.0;
	joint_task->_desired_position = initial_position; 

	vector<Vector3d> last_joint_positions_increment;

	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(45.0, 0.0, -90.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(0.0, 30.0, -30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-45.0, 0.0, 0.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));
	last_joint_positions_increment.push_back(M_PI/180.0*Vector3d(-.0, -30.0, 30.0));

	int motion_steps = 2000;
	int hold_steps = 1500;

	int n_measure_points = last_joint_positions_increment.size();

	Vector3d mean_force = Vector3d::Zero();
	Vector3d mean_moment = Vector3d::Zero();
	MatrixXd local_gravities_cross = MatrixXd(3*n_measure_points, 3);
	VectorXd local_moments = VectorXd(3*n_measure_points);
	Vector3d local_gravity = Vector3d::Zero();
	Vector3d world_gravity = Vector3d(0.0, 0.0, -9.81);
	double g2 = world_gravity.transpose() * world_gravity;
	double estimated_mass = 0;
	Vector3d estimated_com = Vector3d::Zero();


	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	int state = MOVE;
	int current_point = 0;
	int current_motion_step = 0;
	int current_hold_step = 0;

	// runloop = false;

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, robot->_q);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, robot->_dq);
		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_KEY, sensed_force_moment);
		sensed_force_moment -= sensor_bias;

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();
		}

		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		if(state == MOVE)
		{
			if (current_point < n_measure_points)
			{

				if(current_motion_step >= motion_steps)
				{
					if((joint_task->_current_position - joint_task->_desired_position).norm() < 0.001)
					{
						current_hold_step = 0;
						mean_force.setZero();
						mean_moment.setZero();
						state = MEASURE;
						current_point++;
						cout << "measurement " << current_point << endl;
					}
				}
				else
				{
					Vector3d q_increment = last_joint_positions_increment[current_point]/motion_steps;
					
					joint_task->_desired_position.tail(3) = joint_task->_desired_position.tail(3) + q_increment;
					current_motion_step++;
				}
			}
			else
			{
				cout << "calibration finished, exiting controller" << endl;
				state = END;
			}
		}

		else if(state == MEASURE)
		{
			if(current_hold_step > hold_steps/4 && current_hold_step <= 3*hold_steps/4)
			{
				mean_force += sensed_force_moment.head(3);
				mean_moment += sensed_force_moment.tail(3);
			}

			if(current_hold_step >= hold_steps)
			{
				Eigen::Matrix3d R;
				robot->rotation(R,"link7");

				mean_force /= (hold_steps/2);
				mean_moment /= (hold_steps/2);
				double current_mass = (double) (world_gravity.transpose() * R * mean_force) / g2;
				estimated_mass += current_mass;

				local_moments.segment<3>(3*(current_point-1)) = mean_moment;
				local_gravity = R * world_gravity;
				local_gravities_cross.block(3*(current_point-1),0,3,3) = Sai2Model::CrossProductOperator(local_gravity);

				current_motion_step = 0;
				state = MOVE;
				cout << "move to point " << current_point+1 << endl;
			}

			current_hold_step++;
		}

		else if(state == END)
		{
			runloop = false;

			estimated_mass /= n_measure_points;

			MatrixXd A = - estimated_mass * local_gravities_cross;
			VectorXd b = local_moments;

			estimated_com = A.colPivHouseholderQr().solve(b);

			writeXml(estimated_com, estimated_mass);

			cout << endl;
			cout << "estimated mass : " << estimated_mass << endl;
			cout << "estimated com : " << estimated_com.transpose() << endl;
			cout << endl;
		}


		joint_task->computeTorques(joint_task_torques);

		command_torques = joint_task_torques;

		//------ Final torques
		// command_torques << 0, 0, 0, 0, 0, 0, 0;
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
			// cout << "ee orientation :\n" << ori_task->_current_orientation << endl;
			// cout << "command torques : " << command_torques.transpose() << endl;
			// cout << endl;
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


void writeXml(Vector3d object_com, double object_mass)
{
	cout << "write tool properties to file " << path_to_calibration_file << endl;

	ofstream file;
	file.open(path_to_calibration_file);

	if(file.is_open())
	{
		file << "<?xml version=\"1.0\" ?>\n\n";
		file << "<tool name=\"" << tool_name << "\">\n";
		file << "\t<inertial>\n";
		file << "\t\t<origin xyz=\"" << object_com.transpose() << "\">\n";
		file << "\t\t<mass value=\"" << object_mass << "\">\n";
		file << "\t</inertial>\n";
		file << "</tool>" << endl;
		file.close();
	}
	else
	{
		cout << "could not create xml file" << endl;
	}
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


