/*
 * BasePrimitive.h
 *
 *      Author: Mikael Jorda
 */

#ifndef IIWA_FORCE_CONTROL_BASE_PRIMITIVE_H_
#define IIWA_FORCE_CONTROL_BASE_PRIMITIVE_H_

#include <Eigen/Dense>
#include <string>

enum HybridType {single_force_axis, single_position_axis};

class BasePrimitive
{
public:

	BasePrimitive(int dof)
	{
		current_position = Eigen::Vector3d::Zero();
		desired_position = Eigen::Vector3d::Zero();
		current_velocity = Eigen::Vector3d::Zero();
		desired_velocity = Eigen::Vector3d::Zero();
		sensed_force = Eigen::Vector3d::Zero();
		desired_force = Eigen::Vector3d::Zero();

		current_orientation = Eigen::Matrix3d::Zero();
		desired_orientation = Eigen::Matrix3d::Zero();
		orientation_error = Eigen::Vector3d::Zero();
		current_angular_velocity = Eigen::Vector3d::Zero();
		desired_angular_velocity = Eigen::Vector3d::Zero();
		sensed_moment = Eigen::Vector3d::Zero();
		desired_moment = Eigen::Vector3d::Zero();

		kp_pos = Eigen::Matrix3d::Zero();
		kv_pos = Eigen::Matrix3d::Zero();
		kpf_pos = Eigen::Matrix3d::Zero();
		kvf_pos = Eigen::Matrix3d::Zero();
		kif_pos = Eigen::Matrix3d::Zero();
		kp_ori = Eigen::Matrix3d::Zero();
		kv_ori = Eigen::Matrix3d::Zero();
		kpf_ori = Eigen::Matrix3d::Zero();
		kvf_ori = Eigen::Matrix3d::Zero();
		kif_ori = Eigen::Matrix3d::Zero();
		pos_ori_related_forces = Eigen::VectorXd::Zero(6);

		controller_frequency = 1000;
		force_integrated_error.setZero();
		moment_integrated_error.setZero();
		closed_loop_force_control = false;
		closed_loop_moment_control = false;
		force_moment_related_forces = Eigen::VectorXd::Zero(6);

		jacobian = Eigen::MatrixXd::Zero(6,dof);
		projected_jacobian = Eigen::MatrixXd::Zero(6,dof);
		Lambda = Eigen::MatrixXd::Zero(6,6);
		Jbar = Eigen::MatrixXd::Zero(dof,6);
		N = Eigen::MatrixXd::Zero(dof,dof);

		task_force = Eigen::VectorXd::Zero(6);

		linear_hybrid_axis = Eigen::Vector3d::Zero();
		angular_hybrid_axis = Eigen::Vector3d::Zero();
		sigma_force_moment = Eigen::MatrixXd::Zero(6,6);
		sigma_position_orientation = Eigen::MatrixXd::Identity(6,6);

		linear_velocity_saturation = false;
		angular_velocity_saturation = false;

		saturated_linear_velocity.setZero();
		saturated_angular_velocity.setZero();
	}

	// computes the task torques. PD controller for now
	void computeTorques(Eigen::VectorXd& task_joint_torques)
	{
		Eigen::Vector3d force_term;		
		Eigen::Vector3d moment_term;
		Eigen::Vector3d pos_term;
		Eigen::Vector3d ori_term;

		if(closed_loop_force_control)
		{
			force_integrated_error += (sensed_force - desired_force)/controller_frequency;
			force_term = desired_force - kpf_pos * (sensed_force - desired_force) - kif_pos * force_integrated_error - kvf_pos * current_velocity;
		}
		else
		{
			force_term = desired_force;
		}
		if(closed_loop_moment_control)
		{
			moment_integrated_error += (sensed_moment - desired_moment)/controller_frequency;
			moment_term = desired_moment - kpf_ori * (sensed_moment - desired_moment) - kif_ori * moment_integrated_error - kvf_ori * current_velocity;
		}
		else
		{
			moment_term = desired_moment;
		}
		force_moment_related_forces.head(3) = force_term;
		force_moment_related_forces.tail(3) = moment_term;
		

		// PD controller for position part
		if(linear_velocity_saturation)
		{
			// assumes the gains are diagonal matrices
			desired_velocity = - kp_pos * kv_pos.inverse() * (current_position - desired_position);
			for(int i=0; i<3; i++)
			{
				if(desired_velocity(i) > saturated_linear_velocity(i))
				{
					desired_velocity(i) = saturated_linear_velocity(i);
				}
				else if(desired_velocity(i) < -saturated_linear_velocity(i))
				{
					desired_velocity(i) = -saturated_linear_velocity(i);
				}
			}
			pos_term =  -kv_pos*(current_velocity - desired_velocity);
		}
		else
		{
			pos_term = -kp_pos*(current_position - desired_position) - kv_pos*(current_velocity - desired_velocity);
		}

		orientationError(orientation_error, desired_orientation, current_orientation);
		if(angular_velocity_saturation)
		{
			// assumes the gains are diagonal matrices
			desired_angular_velocity = - kp_ori * kv_ori.inverse() * orientation_error;
			for(int i=0; i<3; i++)
			{
				if(desired_angular_velocity(i) > saturated_angular_velocity(i))
				{
					desired_angular_velocity(i) = saturated_angular_velocity(i);
				}
				else if(desired_angular_velocity(i) < -saturated_angular_velocity(i))
				{
					desired_angular_velocity(i) = -saturated_angular_velocity(i);
				}
			}
			ori_term = -kv_ori*(current_angular_velocity - desired_angular_velocity);
		}
		else
		{
			ori_term = -kp_ori*orientation_error - kv_ori*(current_angular_velocity - desired_angular_velocity);
		}
		pos_ori_related_forces.head(3) = pos_term;
		pos_ori_related_forces.tail(3) = ori_term;

		task_force = Lambda * sigma_position_orientation * pos_ori_related_forces + Lambda * sigma_force_moment * force_moment_related_forces;;

		task_joint_torques = projected_jacobian.transpose() * task_force;
	}

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

	void setLinearHybridType(const HybridType t, const Eigen::Vector3d& axis)
	{
		linear_hybrid_axis = axis.normalized();
		if(t == single_force_axis)
		{
			Eigen::Matrix3d tmp = linear_hybrid_axis*linear_hybrid_axis.transpose();
			sigma_force_moment.block(0,0,3,3) = tmp;
			sigma_position_orientation.block(0,0,3,3) = Eigen::Matrix3d::Identity() - tmp;
		}
		else if(t == single_position_axis)
		{
			Eigen::Matrix3d tmp = linear_hybrid_axis*linear_hybrid_axis.transpose();
			sigma_position_orientation.block(0,0,3,3) = tmp;
			sigma_force_moment.block(0,0,3,3) = Eigen::Matrix3d::Identity() - tmp;
		}

	}

	void setAngularHybridType(const HybridType t, const Eigen::Vector3d& axis)
	{
		angular_hybrid_axis = axis.normalized();
		if(t == single_force_axis)
		{
			Eigen::Matrix3d tmp = angular_hybrid_axis*angular_hybrid_axis.transpose();
			sigma_force_moment.block(3,3,3,3) = tmp;
			sigma_position_orientation.block(3,3,3,3) = Eigen::Matrix3d::Identity() - tmp;
		}
		else if(t == single_position_axis)
		{
			Eigen::Matrix3d tmp = angular_hybrid_axis*angular_hybrid_axis.transpose();
			sigma_position_orientation.block(3,3,3,3) = tmp;
			sigma_force_moment.block(3,3,3,3) = Eigen::Matrix3d::Identity() - tmp;
		}

	}
	
	void setLinearPositionControl()
	{
		sigma_force_moment.block(0,0,3,3) = Eigen::Matrix3d::Zero();
		sigma_position_orientation.block(0,0,3,3) = Eigen::Matrix3d::Identity();
	}

	void setLinearForceControl()
	{
		sigma_force_moment.block(0,0,3,3) = Eigen::Matrix3d::Identity();
		sigma_position_orientation.block(0,0,3,3) = Eigen::Matrix3d::Zero();
	}

	void setAngularPositionControl()
	{
		sigma_force_moment.block(3,3,3,3) = Eigen::Matrix3d::Zero();
		sigma_position_orientation.block(3,3,3,3) = Eigen::Matrix3d::Identity();
	}

	void setAngularForceControl()
	{
		sigma_force_moment.block(3,3,3,3) = Eigen::Matrix3d::Identity();
		sigma_position_orientation.block(3,3,3,3) = Eigen::Matrix3d::Zero();
	}

	Eigen::Vector3d getTaskForce()
	{return task_force;}

	Eigen::Vector3d getForceRelatedForces()
	{return force_moment_related_forces;}

	Eigen::Vector3d getPositionRelatedForces()
	{return pos_ori_related_forces;}

	void setClosedLoopForceControl(const double controller_freq)
	{
		closed_loop_force_control = true;
		force_integrated_error.setZero();
		controller_frequency = controller_freq;
	}

	void setOpenLoopForceControl()
	{
		closed_loop_force_control = false;
		force_integrated_error.setZero();
	}

	void setClosedLoopMomentControl(const double controller_freq)
	{
		closed_loop_moment_control = true;
		moment_integrated_error.setZero();
		controller_frequency = controller_freq;
	}

	void setOpenLoopMomentControl()
	{
		closed_loop_moment_control = false;
		moment_integrated_error.setZero();
	}

	void useLinearVelocitySaturation(const bool b, const Eigen::Vector3d& sat = Eigen::Vector3d::Zero())
	{
		linear_velocity_saturation = b;
		saturated_linear_velocity = sat;
		for(int i=0; i<3; i++)
		{
			if(sat(i) < 0)
			{
				std::cout << "WARNING : linear saturation velocity " << i << " should be positive. Set to zero" << std::endl;
				saturated_linear_velocity(i) = 0;
			}
		}
	}

	void useAngularVelocitySaturation(const bool b, const Eigen::Vector3d& sat = Eigen::Vector3d::Zero())
	{
		angular_velocity_saturation = b;
		saturated_angular_velocity = sat;
		for(int i=0; i<3; i++)
		{
			if(sat(i) < 0)
			{
				std::cout << "WARNING : angular saturation velocity " << i << " should be positive. Set to zero" << std::endl;
				saturated_angular_velocity(i) = 0;
			}
		}
	}

	// set gains for all directions
	void setKpPos(const double d)
	{kp_pos = d*Eigen::Matrix3d::Identity();}
	void setKvPos(const double d)
	{kv_pos = d*Eigen::Matrix3d::Identity();}
	void setKpfPos(const double d)
	{kpf_pos = d*Eigen::Matrix3d::Identity();}
	void setKvfPos(const double d)
	{kvf_pos = d*Eigen::Matrix3d::Identity();}
	void setKifPos(const double d)
	{kif_pos = d*Eigen::Matrix3d::Identity();}
	void setKpOri(const double d)
	{kp_ori = d*Eigen::Matrix3d::Identity();}
	void setKvOri(const double d)
	{kv_ori = d*Eigen::Matrix3d::Identity();}
	void setKpfOri(const double d)
	{kpf_ori = d*Eigen::Matrix3d::Identity();}
	void setKvfOri(const double d)
	{kvf_ori = d*Eigen::Matrix3d::Identity();}
	void setKifOri(const double d)
	{kif_ori = d*Eigen::Matrix3d::Identity();}

	// set gains as matrices
	void setKpPos(const Eigen::Matrix3d K)
	{kp_pos = K;}
	void setKvPos(const Eigen::Matrix3d K)
	{kv_pos = K;}
	void setKpfPos(const Eigen::Matrix3d K)
	{kpf_pos = K;}
	void setKvfPos(const Eigen::Matrix3d K)
	{kvf_pos = K;}
	void setKifPos(const Eigen::Matrix3d K)
	{kif_pos = K;}
	void setKpOri(const Eigen::Matrix3d K)
	{kp_ori = K;}
	void setKvOri(const Eigen::Matrix3d K)
	{kv_ori = K;}
	void setKpfOri(const Eigen::Matrix3d K)
	{kpf_ori = K;}
	void setKvfOri(const Eigen::Matrix3d K)
	{kvf_ori = K;}
	void setKifOri(const Eigen::Matrix3d K)
	{kif_ori = K;}

	// state and desired state
	Eigen::Vector3d current_position;
	Eigen::Vector3d desired_position;
	Eigen::Vector3d current_velocity;
	Eigen::Vector3d desired_velocity;
	Eigen::Vector3d sensed_force;
	Eigen::Vector3d desired_force;

	Eigen::Matrix3d current_orientation;
	Eigen::Matrix3d desired_orientation;
	Eigen::Vector3d orientation_error;
	Eigen::Vector3d current_angular_velocity;
	Eigen::Vector3d desired_angular_velocity;
	Eigen::Vector3d sensed_moment;
	Eigen::Vector3d desired_moment;

	// position PD controller gains 
	Eigen::Matrix3d kp_pos;
	Eigen::Matrix3d kv_pos;

	Eigen::Matrix3d kp_ori;
	Eigen::Matrix3d kv_ori;

	Eigen::VectorXd pos_ori_related_forces;

	// force PID controller quantities
	double controller_frequency;
	Eigen::Vector3d force_integrated_error;
	Eigen::Vector3d moment_integrated_error;

	bool closed_loop_force_control;
	bool closed_loop_moment_control;

	Eigen::Matrix3d kpf_pos;
	Eigen::Matrix3d kvf_pos;
	Eigen::Matrix3d kif_pos;

	Eigen::Matrix3d kpf_ori;
	Eigen::Matrix3d kvf_ori;
	Eigen::Matrix3d kif_ori;

	Eigen::VectorXd force_moment_related_forces;
	
	// model
	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd projected_jacobian;
	Eigen::MatrixXd Lambda;
	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd N;

// private:
	Eigen::VectorXd task_force;

	Eigen::Vector3d linear_hybrid_axis;
	Eigen::Vector3d angular_hybrid_axis;
	Eigen::MatrixXd sigma_force_moment;
	Eigen::MatrixXd sigma_position_orientation;

	// velocity saturation
	bool linear_velocity_saturation;
	bool angular_velocity_saturation;
	Eigen::Vector3d saturated_linear_velocity;
	Eigen::Vector3d saturated_angular_velocity;
};

#endif