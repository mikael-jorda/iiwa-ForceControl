/*
 * orientationTask.h
 *
 *      Author: Mikael Jorda
 */

#ifndef IIWA_FORCE_CONTROL_ORIENTATION_TASK_H_
#define IIWA_FORCE_CONTROL_ORIENTATION_TASK_H_

#include <Eigen/Dense>
#include <string>

class OrientationTask
{
public:

	OrientationTask(int dof)
	{
		link_name = "no link defined";

		current_orientation = Eigen::Matrix3d::Identity(3,3);
		desired_orientation = Eigen::Matrix3d::Identity(3,3);

		current_angular_velocity = Eigen::Vector3d::Zero(3);
		desired_angular_velocity = Eigen::Vector3d::Zero(3);

		kp = Eigen::Matrix3d::Zero();
		kv = Eigen::Matrix3d::Zero();
		kpf = Eigen::Matrix3d::Zero();
		kvf = Eigen::Matrix3d::Zero();

		jacobian = Eigen::MatrixXd::Zero(3,dof);
		projected_jacobian = Eigen::MatrixXd::Zero(3,dof);
		Lambda = Eigen::MatrixXd::Zero(3,3);
		Jbar = Eigen::MatrixXd::Zero(dof,3);
		N = Eigen::MatrixXd::Zero(dof,dof);

		task_force = Eigen::Vector3d::Zero();
		orientation_error = Eigen::Vector3d::Zero();

		desired_moments = Eigen::Vector3d::Zero();
		sensed_moments = Eigen::Vector3d::Zero();
		force_control = false;
	}

	// computes the task torques. PD controller for now
	void computeTorques(Eigen::VectorXd& task_joint_torques)
	{

		if(force_control)
		{
			task_force = Lambda*(-kpf*(sensed_moments - desired_moments) - kvf*(current_angular_velocity - desired_angular_velocity ));
		}
		else
		{
			orientationError(orientation_error, desired_orientation, current_orientation);
			task_force = Lambda*(-kp*orientation_error - kv*(current_angular_velocity - desired_angular_velocity ));
		}

		task_joint_torques = projected_jacobian.transpose()*task_force;
	}

	// set kp for all the joints
	void setKp(const double d)
	{
		kp = d*Eigen::Matrix3d::Identity();
	}

	// set kv for all the joints
	void setKv(const double d)
	{
		kv = d*Eigen::Matrix3d::Identity();
	}

	// set kpf for all the joints
	void setKpf(const double d)
	{
		kpf = d*Eigen::Matrix3d::Identity();
	}

	// set kpf for all the joints
	void setKvf(const double d)
	{
		kvf = d*Eigen::Matrix3d::Identity();
	}

	Eigen::Vector3d getOrientationError()
	{return orientation_error;}

	Eigen::Vector3d getTaskForce()
	{return task_force;}

	std::string link_name;

	Eigen::Matrix3d current_orientation;
	Eigen::Matrix3d desired_orientation;

	Eigen::Vector3d desired_moments;
	Eigen::Vector3d sensed_moments;

	Eigen::Vector3d current_angular_velocity;
	Eigen::Vector3d desired_angular_velocity;

	Eigen::Matrix3d kp;
	Eigen::Matrix3d kv;
	Eigen::Matrix3d kpf;
	Eigen::Matrix3d kvf;

	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd projected_jacobian;
	Eigen::MatrixXd Lambda;
	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd N;

	bool force_control;


private:

	void orientationError(Eigen::Vector3d& delta_phi,
	const Eigen::Matrix3d& desired_orientation,
	const Eigen::Matrix3d& current_orientation)
	{
	// check that the matrices are valid rotations
		Eigen::Matrix3d Q1 = desired_orientation*desired_orientation.transpose() - Eigen::Matrix3d::Identity();
		Eigen::Matrix3d Q2 = current_orientation*current_orientation.transpose() - Eigen::Matrix3d::Identity();
		if(Q1.norm() > 0.0001 || Q2.norm() > 0.0001)
		{
			throw std::invalid_argument("Invalid rotation matrices in ModelInterface::orientationError");
			return;
		}
		else
		{
			Eigen::Vector3d rc1 = current_orientation.block<3,1>(0,0);
			Eigen::Vector3d rc2 = current_orientation.block<3,1>(0,1);
			Eigen::Vector3d rc3 = current_orientation.block<3,1>(0,2);
			Eigen::Vector3d rd1 = desired_orientation.block<3,1>(0,0);
			Eigen::Vector3d rd2 = desired_orientation.block<3,1>(0,1);
			Eigen::Vector3d rd3 = desired_orientation.block<3,1>(0,2);
			delta_phi = -1.0/2.0*(rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
		}
	}

	Eigen::Vector3d task_force;
	Eigen::Vector3d orientation_error;
};

#endif