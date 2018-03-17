// This example application runs a controller for the IIWA

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "primitives/RedundantArmMotion.h"

#include <iostream>
#include <fstream>
#include <string>

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

// int simulation = true;
int simulation = false;

// write xml file
const string path_to_bias_file = "../../util-fsensor_calibration/calibration_files/sensor_bias.xml";
// const string path_to_bias_file = "sensor_bias.xml";
void writeXml(VectorXd sensor_bias);

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
	control_frame.translation() = Vector3d(0.0, 0.0, -0.081);
	auto motion_primitive = new Sai2Primitives::RedundantArmMotion(robot, control_link, control_frame);

	Vector3d initial_position;
	robot->position(initial_position, control_link, control_frame.translation());

	VectorXd motion_primitive_torques = VectorXd::Zero(dof);

	motion_primitive->_posori_task->_kp_pos = 50.0;
	motion_primitive->_posori_task->_kv_pos = 10.0;
	motion_primitive->_posori_task->_kp_ori = 80.0;
	motion_primitive->_posori_task->_kv_ori = 14.0;

	motion_primitive->_joint_task->_kp = 0.0;
	motion_primitive->_joint_task->_kv = 5.0;

	// bias vector
	VectorXd bias = VectorXd::Zero(6);
	Matrix3d initial_orientation = Matrix3d::Zero();

	initial_orientation << 0,-1,0,
						   -1,0,0,
						   0,0,-1;

	Matrix3d desired_orientation = initial_orientation;
	motion_primitive->_desired_orientation = desired_orientation;

	vector<Vector3d> rotation_axis;
	vector<double> rotation_angles;

	rotation_axis.push_back(Vector3d(0.0, 0.0, 1.0));
	rotation_angles.push_back(-3.0*M_PI/4.0);

	rotation_axis.push_back(Vector3d(-sqrt(2)/2.0, -sqrt(2)/2.0, 0));
	rotation_angles.push_back(M_PI/2.0);

	rotation_axis.push_back(Vector3d(0, 0, 1));
	rotation_angles.push_back(M_PI/2.0);

	rotation_axis.push_back(Vector3d(0, 0, 1));
	rotation_angles.push_back(M_PI/2.0);

	rotation_axis.push_back(Vector3d(0, 0, 1));
	rotation_angles.push_back(M_PI/2.0);

	rotation_axis.push_back(Vector3d(sqrt(2)/2.0, -sqrt(2)/2.0, 0));
	rotation_angles.push_back(M_PI/2.0);

	int motion_steps = 2000;
	int hold_steps = 1500;

	int n_measure_points = rotation_angles.size();

	VectorXd bias_force_moments = VectorXd::Zero(6);
	VectorXd bias_force_moments_tmp = VectorXd::Zero(6);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(sighandler);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::AngleAxisd AA = Eigen::AngleAxisd(M_PI/4, Vector3d(1,0,0));
	Eigen::AngleAxisd BB = Eigen::AngleAxisd(M_PI/4, Vector3d(1,0,0));

	int state = MOVE;
	int current_point = 0;
	int current_motion_step = 0;
	int current_hold_step = 0;

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

		if(state == MOVE)
		{
			if(current_point == 0)
			{
				cout << "error : " << (motion_primitive->_posori_task->_current_orientation - initial_orientation).norm() << endl;
				if((motion_primitive->_posori_task->_current_orientation - initial_orientation).norm() < 0.05)
				{
					current_hold_step = 0;
					current_point = 1;
					cout << "move to point " << current_point << endl;
				}
			}
			else if (current_point <= n_measure_points)
			{


				if(current_motion_step >= motion_steps)
				{
					if((motion_primitive->_posori_task->_current_orientation - motion_primitive->_posori_task->_desired_orientation).norm() < 0.05)
					{
						current_hold_step = 0;
						bias_force_moments_tmp.setZero(6);
						state = MEASURE;
						cout << "measurement " << current_point << endl;
						current_point++;
					}
				}
				else
				{
					double angle = rotation_angles[current_point-1];
					Vector3d axis = rotation_axis[current_point-1];
					axis.normalize();

					Matrix3d R_increment = AngleAxisd(angle/motion_steps, axis).toRotationMatrix();

					Matrix3d tmp = motion_primitive->_desired_orientation;
					desired_orientation = tmp * R_increment;

					motion_primitive->_desired_orientation = desired_orientation;
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
				bias_force_moments_tmp += sensed_force_moment;
			}

			if(current_hold_step >= hold_steps)
			{
				bias_force_moments_tmp /= (hold_steps/2.0);
				bias_force_moments += bias_force_moments_tmp;

				current_motion_step = 0;
				state = MOVE;
				cout << "move to point " << current_point << endl;
			}

			current_hold_step++;
		}

		else if(state == END)
		{
			runloop = false;

			bias_force_moments /= n_measure_points;

			cout << endl;
			cout << "bias force : " << bias_force_moments.transpose() << endl;
			cout << endl;
			writeXml(bias_force_moments);
		}


		motion_primitive->computeTorques(motion_primitive_torques);
		command_torques = motion_primitive_torques;

		//------ Final torques
		// command_torques << 0, 0, 0, 0, 0, 0, 0;
		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		if(controller_counter % 500 == 0)
		{
			// cout << "ee orientation :\n" << motion_primitive->_posori_task->_current_orientation << endl;
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


void writeXml(VectorXd sensor_bias)
{
	if(sensor_bias.size() != 6)
	{
		cout << "bias should be a vector of length 6\nXml file not written" << endl;
		return;
	}

	cout << "write bias to file " << path_to_bias_file << endl;

	ofstream file;
	file.open(path_to_bias_file);

	if(file.is_open())
	{
		file << "<?xml version=\"1.0\" ?>\n";
		file << "<force_bias value=\"" << sensor_bias.transpose() << "\"/>\n";
		file.close();
	}
	else
	{
		cout << "could not create xml file" << endl;
	}
	


}

