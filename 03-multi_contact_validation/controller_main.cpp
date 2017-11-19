// This example application runs a controller for the IIWA

#include "model/ModelInterface.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

#include "tasks/OrientationTask.h"
#include "tasks/HybridPositionTask.h"

#include <signal.h>
bool runloop = true;
void stop(int){runloop = false;}

using namespace std;

const string world_file = "../resources/03-multi_contact_validation/world.urdf";
const string robot_file = "../resources/03-multi_contact_validation/kuka_iiwa_floating_base.urdf";
const string robot_name = "Kuka-IIWA";

unsigned long long controller_counter = 0;

// redis keys:
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::iiwaForceControl::iiwaBot::actuators::fgc";
// - read:
const std::string JOINT_ANGLES_KEY  = "sai2::iiwaForceControl::iiwaBot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::iiwaForceControl::iiwaBot::sensors::dq";
// - debug
const std::string BASE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::base_force_sensor::force";
const std::string EE_FORCE_SENSOR_FORCE_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::sensors::ee_force_sensor::force";

 void sighandler(int sig)
 { runloop = false; }

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

	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);

	// Eigen::Vector3d world_gravity = Eigen::Vector3d(0.0,0.0,-1.0);

	int dof = robot->dof();
	int actuated_dof = dof-6;	
	Eigen::VectorXd actuated_joint_positions(actuated_dof), actuated_joints_velocities(actuated_dof);
	Eigen::VectorXd actuated_joint_torques(actuated_dof);
	
	// read from Redis
	redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, actuated_joint_positions);
	redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, actuated_joints_velocities);
	robot->_q.tail(actuated_dof) = actuated_joint_positions;
	robot->_dq.tail(actuated_dof) = actuated_joints_velocities;

	////////////////////////////////////////////////
	///    Prepare the different controllers   /////
	////////////////////////////////////////////////
	robot->updateModel();

	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof-6);
	Eigen::MatrixXd N_prec;

	// Joint control
	Eigen::VectorXd joint_task_desired_position(actuated_dof), joint_task_torques(actuated_dof);

	double joint_kv = 20.0;

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	// timer.setThreadHighPriority();  // make timing more accurate. requires running executable as sudo.
	timer.setCtrlCHandler(stop);    // exit while loop on ctrl-c
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	Eigen::VectorXd base_sensed_force = Eigen::VectorXd::Zero(3);
	Eigen::VectorXd ee_sensed_force = Eigen::VectorXd::Zero(3);

	// prepare virtual linkage model framework
	// assume forces and momenta at each contact
	Eigen::Matrix3d Rb, Ree, Robject;
	Eigen::MatrixXd R_0_loc = Eigen::MatrixXd::Zero(12,12);

	vector<string> link_names;
	link_names.push_back("base_link");
	link_names.push_back("link6");

	vector<Eigen::Vector3d> pos_in_links;
	pos_in_links.push_back(Eigen::Vector3d(0,0,0));
	pos_in_links.push_back(Eigen::Vector3d(0.0,0,0.05));

	vector<Model::ContactNature> contact_natures;
	contact_natures.push_back(Model::SurfaceContact);
	contact_natures.push_back(Model::SurfaceContact);

	Eigen::Vector3d center_point;
	Eigen::Vector3d pb, pee;

	Eigen::MatrixXd G_tmp, P, Q, G, G_bar;
	Eigen::MatrixXd G11, G12, G13, G21, G22, G23, G13_inv;
	Eigen::MatrixXd U, Ns, UN, UN_bar, Nf;
	U = Eigen::MatrixXd::Zero(actuated_dof, dof);
	U.block(0,dof - actuated_dof, actuated_dof, actuated_dof) = Eigen::MatrixXd::Identity(actuated_dof, actuated_dof);
	Nf = Eigen::MatrixXd::Zero(dof,dof);
	Ns = Eigen::MatrixXd::Zero(dof,dof);
	UN = Eigen::MatrixXd::Zero(actuated_dof, dof);
	UN_bar = Eigen::MatrixXd::Zero(dof, actuated_dof);

	Eigen::MatrixXd Js, Js_bar, Jf, Jf_s, Jf_s_bar, Jpc, Jac, Jbase, Jee;
	Jpc = Eigen::MatrixXd::Zero(7,dof);
	Jac = Eigen::MatrixXd::Zero(5,dof);
	Js = Eigen::MatrixXd::Zero(6,dof);
	Js = Eigen::MatrixXd::Zero(dof,6);
	Jf = Eigen::MatrixXd::Zero(6,dof);

	Eigen::MatrixXd Jtot, Jtot_bar;

	Eigen::VectorXd joint_gravity, ptot, Ftot, ftot, ff;

	ff = Eigen::VectorXd::Zero(6);
	// ff(5) = 50;


	// fill in P and Q
	P = Eigen::MatrixXd::Zero(12,12);
	Eigen::VectorXd p_fill = Eigen::VectorXd::Zero(12);
	p_fill << 6, 7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 11;
	for(int i=0; i<12; i++)
	{
		// P(i,p_fill(i)) = 1;
		P(p_fill(i),i) = 1;
	}

	Q = Eigen::MatrixXd::Zero(12,12);
	Eigen::VectorXd q_fill = Eigen::VectorXd::Zero(12);
	q_fill << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11;
	for(int i=0; i<12; i++)
	{
		Q(i,q_fill(i)) = 1;
	}

	// while window is open:
	while (runloop) {

		// wait for next scheduled loop
		timer.waitForNextLoop();

		// read from Redis
		redis_client.getEigenMatrixDerived(JOINT_ANGLES_KEY, actuated_joint_positions);
		redis_client.getEigenMatrixDerived(JOINT_VELOCITIES_KEY, actuated_joints_velocities);
		redis_client.getEigenMatrixDerived(BASE_FORCE_SENSOR_FORCE_KEY, base_sensed_force);
		redis_client.getEigenMatrixDerived(EE_FORCE_SENSOR_FORCE_KEY, ee_sensed_force);

		robot->_q.tail(actuated_dof) = actuated_joint_positions;
		robot->_dq.tail(actuated_dof) = actuated_joints_velocities;

		// update the model 20 times slower
		if(controller_counter%20 == 0)
		{
			robot->updateModel();

			// robot->position(pb, link_names[0], pos_in_links[0]);
			// robot->position(pee, link_names[1], pos_in_links[1]);
			robot->rotation(Rb, link_names[0]);
			robot->rotation(Ree, link_names[1]);
			R_0_loc.block<3,3>(0,0) = Rb;
			R_0_loc.block<3,3>(3,3) = Ree;
			R_0_loc.block<3,3>(6,6) = Rb;
			R_0_loc.block<3,3>(9,9) = Ree;
			robot->GraspMatrixAtGeometricCenter(G_tmp, Robject, center_point, link_names, pos_in_links, contact_natures);
			G_tmp = G_tmp*R_0_loc;
			G = Q*G_tmp*P;
			G_bar = G.inverse();

			G11 = G_bar.block<5,6>(0,0);
			G12 = G_bar.block<5,1>(0,6);
			G13 = G_bar.block<5,5>(0,7);
			G21 = G_bar.block<7,6>(5,0);
			G22 = G_bar.block<7,1>(5,6);
			G23 = G_bar.block<7,5>(5,7);
			G13_inv = G13.inverse();

			robot->J_0(Jbase, link_names[0], pos_in_links[0]);
			robot->J_0(Jee, link_names[1], pos_in_links[1]);

			// std::cout << "Jee : \n" << Jee << endl << endl;

			Jpc.block(0,0,3,dof) = Jbase.block(0,0,3,dof);
			Jpc.block(3,0,3,dof) = Jee.block(0,0,3,dof);
			Jpc.block(6,0,1,dof) = Jee.block(5,0,1,dof);

			Jac.block(0,0,3,dof) = Jbase.block(3,0,3,dof);
			Jac.block(3,0,2,dof) = Jee.block(3,0,2,dof);

			Js = (G21.transpose() - G11.transpose()*G13_inv.transpose()*G23.transpose())*Jpc;
			Js_bar = robot->_M_inv*Js.transpose()*(Js*robot->_M_inv*Js.transpose()).inverse();
			Ns = Eigen::MatrixXd::Identity(dof,dof) - Js_bar*Js;

			UN = U*Ns;
			UN_bar = robot->_M_inv*UN.transpose()*(UN*robot->_M_inv*UN.transpose()).inverse();

			Jf.block(0,0,5,dof) = Jac + G13_inv.transpose()*G23.transpose()*Jpc;
			Jf.block(5,0,1,dof) = (G22.transpose() - G12.transpose()*G13_inv.transpose()*G23.transpose())*Jpc;
			Jf_s = Jf*Ns;
			Jf_s_bar = robot->_M_inv*Jf_s.transpose()*(Jf_s*robot->_M_inv*Jf_s.transpose()).inverse();

			Nf = Eigen::MatrixXd::Identity(dof,dof) - Jf_s_bar*Jf_s;

			Jtot = Jf_s;
			// Jtot.block(0,0,6,dof) = Jf_s;
			// Jtot.block(6,0,dof,dof) = Nf*Ns;
			Jtot_bar = robot->_M_inv*Jtot.transpose()*(Jtot*robot->_M_inv*Jtot.transpose()).inverse();

		}
		// robot->gravityVector(joint_gravity, world_gravity);
		robot->gravityVector(joint_gravity);
		ptot = Jtot_bar.transpose()*Ns.transpose()*joint_gravity;
		// ptot = Jtot_bar.transpose()*(Ns.transpose() - Eigen::MatrixXd::Identity(dof,dof))*joint_gravity;

		// ftot = Jtot_bar.transpose()*Jf_s.transpose()*ff;

		Ftot = ptot;

		actuated_joint_torques = UN_bar.transpose() * Jtot.transpose() * Ftot;

		if(controller_counter %500 == 0)
		{
			// std::cout << "ff \t" << ff.transpose() << endl;
			// std::cout << "joint_gravity \t" << joint_gravity.transpose() << endl;
			// std::cout << "joint_gravity \t" << joint_gravity.tail(actuated_dof).transpose() << endl;
			// std::cout << "m inv \n" << robot->_M_inv << endl;
			// std::cout << "Nf \n" << Nf << endl;
			// std::cout << "Jf \n" << Jf << endl;
			// std::cout << "Jf_s \n" << Jf_s << endl;
			// std::cout << "Jf_s bar \n" << Jf_s_bar << endl;
			// std::cout << "Ns \n" << Ns << endl;
			// std::cout << "Jtot bar \n" << Jtot_bar << endl;
			// std::cout << "JF_s \n" << Jf_s << endl;
			// std::cout << "ptot \t" << ptot.transpose() << endl;
			// std::cout << "ftot \t" << ftot.transpose() << endl;
			// std::cout << "Ftot \t" << Ftot.transpose() << endl;
			// std::cout << "q : \t" << robot->_q.transpose() << endl;
			// std::cout << "p base : \t" << (pb).transpose() << endl;
			// std::cout << "p ee : \t" << (pee).transpose() << endl;
			// std::cout << "p base-ee : \t" << (pee-pb).transpose() << endl;
			// std::cout << "R object : \n" << Robject << endl;
			// std::cout << "R base : \n" << Rb << endl;
			// std::cout << "R ee : \n" << Ree << endl;
			// std::cout << "G13 : \n" << G13 << endl;
			// std::cout << "G13 inverse : \n" << G13.inverse() << endl << endl;
		}


		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		// robot->gravityVector(joint_gravity);

		//------ Final torques
		// command_torques = actuated_joint_torques;
		command_torques = actuated_joint_torques - joint_gravity.tail(actuated_dof);
		// command_torques = Eigen::VectorXd::Zero(actuated_dof);

		redis_client.setEigenMatrixDerived(JOINT_TORQUES_COMMANDED_KEY, command_torques);

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
