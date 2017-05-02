/*
 * HybridPositionTask.h
 *
 *      Author: Mikael Jorda
 */

#ifndef IIWA_FORCE_CONTROL_HYBRIDPOSITION_TASK_H_
#define IIWA_FORCE_CONTROL_HYBRIDPOSITION_TASK_H_

#include <Eigen/Dense>
#include <string>

class HybridPositionTask
{
public:

	HybridPositionTask(int dof)
	{
		link_name = "no link defined";
		pos_in_link = Eigen::Vector3d(0,0,0);

		current_position = Eigen::Vector3d::Zero();
		desired_position = Eigen::Vector3d::Zero();

		current_force = Eigen::Vector3d::Zero();
		desired_force = Eigen::Vector3d::Zero();

		current_velocity = Eigen::Vector3d::Zero();
		desired_velocity = Eigen::Vector3d::Zero();

		kp = Eigen::Matrix3d::Zero();
		kv = Eigen::Matrix3d::Zero();

		jacobian = Eigen::MatrixXd::Zero(3,dof);
		projected_jacobian = Eigen::MatrixXd::Zero(3,dof);
		Lambda = Eigen::MatrixXd::Zero(3,3);
		Jbar = Eigen::MatrixXd::Zero(dof,3);
		N = Eigen::MatrixXd::Zero(dof,dof);

		task_force = Eigen::Vector3d::Zero();

		force_related_forces = Eigen::Vector3d::Zero();
		position_related_forces = Eigen::Vector3d::Zero();

		force_axis = Eigen::Vector3d::Zero();
		sigma_force = Eigen::Matrix3d::Zero();
		sigma_position = Eigen::Matrix3d::Identity();
	}

	// computes the task torques. PD controller for now
	void computeTorques(Eigen::VectorXd& task_joint_torques)
	{
		// open loop force controller for now
		force_related_forces = sigma_force * desired_force;

		// PD controller for position part
		position_related_forces = Lambda * (sigma_position * (-kp*(current_position - desired_position) - kv*(current_velocity - desired_velocity)));

		task_force = force_related_forces + position_related_forces;

		task_joint_torques = projected_jacobian.transpose() * task_force;
	}

	void setForceAxis(const Eigen::Vector3d& axis)
	{
		force_axis = axis.normalized();

		sigma_force = force_axis*force_axis.transpose();
		sigma_position = Eigen::Matrix3d::Identity() - sigma_force;
	}

	Eigen::Vector3d getTaskForce()
	{return task_force;}

	Eigen::Vector3d getForceRelatedForces()
	{return force_related_forces;}

	Eigen::Vector3d getPositionRelatedForces()
	{return position_related_forces;}

	Eigen::Vector3d getForceAxis()
	{return force_axis;}

	std::string link_name;
	Eigen::Vector3d pos_in_link;

	Eigen::Vector3d current_position;
	Eigen::Vector3d desired_position;

	Eigen::Vector3d current_velocity;
	Eigen::Vector3d desired_velocity;

	Eigen::Vector3d current_force;
	Eigen::Vector3d desired_force;

	Eigen::Matrix3d kp;
	Eigen::Matrix3d kv;

	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd projected_jacobian;
	Eigen::MatrixXd Lambda;
	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd N;

private:
	Eigen::Vector3d task_force;

	Eigen::Vector3d position_related_forces;
	Eigen::Vector3d force_related_forces;

	Eigen::Vector3d force_axis;
	Eigen::Matrix3d sigma_force;
	Eigen::Matrix3d sigma_position;
};

#endif