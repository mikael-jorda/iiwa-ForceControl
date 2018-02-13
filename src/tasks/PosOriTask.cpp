/*
 * PosOriTask.cpp
 *
 *      Author: Mikael Jorda
 */

#include "PosOriTask.h"

#include <stdexcept>



PosOriTask::PosOriTask(Sai2Model::Sai2Model* robot, 
					const std::string link_name, 
					const Eigen::Affine3d control_frame, 
					const Eigen::Affine3d sensor_frame)
{
	_robot = robot;
	_link_name = link_name;
	_T_link_control = control_frame;

	int dof = _robot->_dof;

	// motion
	_robot->position(_current_position, _link_name, _T_link_control.translation());
	_robot->position(_desired_position, _link_name, _T_link_control.translation());
	_robot->rotation(_current_orientation, _link_name);
	_robot->rotation(_desired_orientation, _link_name);

	_current_linear_velocity.setZero();
	_desired_linear_velocity.setZero();
	_current_angular_velocity.setZero();
	_desired_angular_velocity.setZero();

	_kp_pos = 0;
	_kv_pos = 0;
	_ki_pos = 0;
	_kp_ori = 0;
	_kv_ori = 0;
	_ki_ori = 0;

	_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_orientation_error.setZero();

	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_orientation = Eigen::Matrix3d::Identity();

	// force
	_T_link_sensor = sensor_frame;  
	_T_control_sensor = _T_link_control.inverse() * _T_link_sensor;

	_desired_force.setZero();
	_sensed_force.setZero();
	_desired_moment.setZero();
	_sensed_moment.setZero();

	_kp_force = 0;
	_kv_force = 0;
	_ki_force = 0;
	_kp_moment = 0;
	_kv_moment = 0;
	_ki_moment = 0;

	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();

	_sigma_force.setZero();
	_sigma_moment.setZero();

	_closed_loop_force_control = false;
	_closed_loop_moment_control = false;

	// passivity
	_passivity_enabled = true;
	_Rc_force = 1;
	_Rc_moment = 1;
	_PO_force = 0;
	_PO_moment = 0;

	_PO_buffer_force.setZero(5);
	_PO_buffer_moment.setZero(5);

	// model
	_task_force.setZero(6);

	_jacobian.setZero(6,dof);
	_projected_jacobian.setZero(6,dof);
	_Lambda.setZero(6,6);
	_Jbar.setZero(dof,6);
	_N.setZero(dof,dof);
	_N_prec = Eigen::MatrixXd::Identity(dof,dof);

	_first_iteration = true;
}

void PosOriTask::updateTaskModel(const Eigen::MatrixXd N_prec)
{
	if(N_prec.rows() != N_prec.cols())
	{
		throw std::invalid_argument("N_prec matrix not square in PosOriTask::updateTaskModel\n");
	}
	if(N_prec.rows() != _robot->_dof)
	{
		throw std::invalid_argument("N_prec matrix size not consistent with robot dof in PosOriTask::updateTaskModel\n");
	}

	_N_prec = N_prec;

	_robot->J_0(_jacobian, _link_name, _T_link_control.translation());
	_projected_jacobian = _jacobian * _N_prec;
	_robot->operationalSpaceMatrices(_Lambda, _Jbar, _N, _projected_jacobian, _N_prec);

}


void PosOriTask::computeTorques(Eigen::VectorXd& task_joint_torques)
{

	// get time since last call for the I term
	_t_curr = std::chrono::system_clock::now();
	if(_first_iteration)
	{
		_t_prev = std::chrono::system_clock::now();
		_first_iteration = false;
	}
	_t_diff = _t_curr - _t_prev;
	double dt = _t_diff.count();

	Eigen::Vector3d force_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d position_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d moment_related_force = Eigen::Vector3d::Zero();
	Eigen::Vector3d orientation_related_force = Eigen::Vector3d::Zero();

	// update velocity and angular velocity for damping term
	_current_linear_velocity = _projected_jacobian.block(0,0,3,_robot->_dof) * _robot->_dq;
	_current_angular_velocity = _projected_jacobian.block(3,0,3,_robot->_dof) * _robot->_dq;


	// force related terms
	if(_closed_loop_force_control)
	{
		// update the integrated error
		_integrated_force_error += (_sensed_force - _desired_force) * dt;

		// compute the feedback term
		Eigen::Vector3d force_feedback_term = - _kp_force * (_sensed_force - _desired_force) - _ki_force * _integrated_force_error;

		// run passivity observer and controller
		if(_passivity_enabled)
		{
			// compute PO value
			double tmp_Rc = _Rc_force;
			double p_input = _desired_force.transpose() * (_sigma_force * force_feedback_term);
			double p_output = tmp_Rc * force_feedback_term.transpose()*(_sigma_force * force_feedback_term);

			_PO_force += (p_input - p_output)*dt;

			// update the buffer and reset if needed
			for(int i=4; i>0; i--)
			{
				_PO_buffer_force(i) = _PO_buffer_force(i-1);
			}
			_PO_buffer_force(0) = p_input - p_output;

			if(_PO_buffer_force(0) > _PO_buffer_force(1) &&
					_PO_buffer_force(1) > _PO_buffer_force(2) &&
					_PO_buffer_force(2) < _PO_buffer_force(3) &&
					_PO_buffer_force(3) < _PO_buffer_force(4) &&
					_PO_force < 0)
			{
				_PO_force = _PO_buffer_force(0) + _PO_buffer_force(1);
			}

			// required values for PC trick
			double Vc_square = force_feedback_term.transpose() * _sigma_force * force_feedback_term;
			Eigen::Vector3d deltaf = _sensed_force - _desired_force;
			double deltaf_square = deltaf.transpose()*_sigma_force*deltaf;

			if(_PO_force < 0)
			{
				// update RC value
				_Rc_force = (_Rc_force + (1-2*atan(-0.1*_PO_force)/M_PI)*deltaf_square) / (1+deltaf_square);
				
				if(_Rc_force < 0) // should never happen
				{
					_Rc_force = 0;
				}
				std::cout << "Rc force : " << _Rc_force << std::endl;
			}
			else
			{

				_Rc_force = (_Rc_force + 1.0*deltaf_square)/(1+deltaf_square);
				if(_Rc_force > 1) // should never happen
				{
					_Rc_force = 1;
				}
			}
			// update PO value to take dissipated energy into account
			_PO_force += (tmp_Rc-_Rc_force)*Vc_square*dt;

		}

		// compute the final contribution
		force_related_force = _sigma_force * (_desired_force + _Rc_force * force_feedback_term - _kv_force * _current_linear_velocity);
	}
	else
	{
		force_related_force = _sigma_force * _desired_force;
	}

	// find effective mass in this direction
	double eff_mass_force = 1;

	if(force_related_force.norm() > 0.001)
	{
		// eff_mass_force = (double) (force_related_force.transpose() * _Lambda.block(0,0,3,3) * force_related_force)/(force_related_force.norm());
		// eff_mass_force = (double) (force_related_force.transpose() * _Lambda.inverse().block(0,0,3,3) * force_related_force)/(force_related_force.norm());
		eff_mass_force = _Lambda(3,3);
	}
	// force_related_force = force_related_force / eff_mass_force; 


	// moment related terms
	if(_closed_loop_moment_control)
	{
		// update the integrated error
		_integrated_moment_error += (_sensed_moment - _desired_moment) * dt;

		// compute the feedback term
		Eigen::Vector3d moment_feedback_term = - _kp_moment * (_sensed_moment - _desired_moment) - _ki_moment * _integrated_moment_error;

		// run passivity observer and controller
		if(_passivity_enabled)
		{
			// compute PO value
			double tmp_Rc = _Rc_moment;
			double p_input = _desired_moment.transpose() * (_sigma_moment * moment_feedback_term);
			double p_output = tmp_Rc * moment_feedback_term.transpose()*(_sigma_moment * moment_feedback_term);

			_PO_moment += (p_input - p_output)*dt;

			// update the buffer and reset if needed
			for(int i=4; i>0; i--)
			{
				_PO_buffer_moment(i) = _PO_buffer_moment(i-1);
			}
			_PO_buffer_moment(0) = p_input - p_output;

			if(_PO_buffer_moment(0) > _PO_buffer_moment(1) &&
					_PO_buffer_moment(1) > _PO_buffer_moment(2) &&
					_PO_buffer_moment(2) < _PO_buffer_moment(3) &&
					_PO_buffer_moment(3) < _PO_buffer_moment(4) &&
					_PO_moment < 0)
			{
				_PO_moment = _PO_buffer_moment(0) + _PO_buffer_moment(1);
			}

			// required values for PC trick
			double Vc_square = moment_feedback_term.transpose() * _sigma_moment * moment_feedback_term;
			Eigen::Vector3d deltaf = _sensed_moment - _desired_moment;
			double deltaf_square = deltaf.transpose()*_sigma_moment*deltaf;

			if(_PO_moment < 0)
			{
				// update RC value
				_Rc_moment = (_Rc_moment + (1-2*atan(-0.1*_PO_moment)/M_PI)*deltaf_square) / (1+deltaf_square);
				
				if(_Rc_moment < 0) // should never happen
				{
					_Rc_moment = 0;
				}
				std::cout << "Rc moment : " << _Rc_moment << std::endl;
			}
			else
			{

				_Rc_moment = (_Rc_moment + 1.0*deltaf_square)/(1+deltaf_square);
				if(_Rc_moment > 1) // should never happen
				{
					_Rc_moment = 1;
				}
			}
			// update PO value to take dissipated energy into account
			_PO_moment += (tmp_Rc-_Rc_moment)*Vc_square*dt;

		}

		// compute the final contribution
		moment_related_force = _sigma_moment * (_desired_moment + _Rc_moment * moment_feedback_term - _kv_moment * _current_angular_velocity);

	}
	else
	{
		moment_related_force = _sigma_moment * _desired_moment;
	}

	// find effective mass in this direction
	double eff_mass_moment = 1;

	if(moment_related_force.norm() > 0.001)
	{
		eff_mass_moment = (double) (moment_related_force.transpose() * _Lambda.block(0,0,3,3) * moment_related_force)/(moment_related_force.norm());
	}
	moment_related_force /= eff_mass_moment; 

	// linear motion related terms
	// get curent position for P term
	_robot->position(_current_position, _link_name, _T_link_control.translation());

	// update integrated error for I term
	_integrated_position_error += (_current_position - _desired_position) * dt;

	// final contribution
	position_related_force = _sigma_position * ( -_kp_pos*(_current_position - _desired_position) - _kv_pos*(_current_linear_velocity - _desired_linear_velocity) - _ki_pos*_integrated_position_error);


	// angular motion related terms
	// get curent position and orientation error for P term
	_robot->rotation(_current_orientation, _link_name);
	_current_orientation = _current_orientation * _T_link_control.linear(); // orientation of compliant frame in robot frame
	Sai2Model::orientationError(_orientation_error, _desired_orientation, _current_orientation);

	// update integrated error for I term
	_integrated_orientation_error += _orientation_error * dt;

	// final contribution
	orientation_related_force = _sigma_orientation * ( -_kp_ori*_orientation_error - _kv_ori*(_current_angular_velocity - _desired_angular_velocity) - _ki_ori*_integrated_orientation_error);


	// compute task force
	_task_force.head(3) = position_related_force + _Lambda.inverse().block(0,0,3,3) * force_related_force;
	_task_force.tail(3) = orientation_related_force + moment_related_force;
	_task_force = _Lambda*_task_force;

	// compute task torques
	task_joint_torques = _projected_jacobian.transpose()*_task_force;

	// update previous time
	_t_prev = _t_curr;
}

void PosOriTask::setForceSensorFrame(const std::string link_name, const Eigen::Affine3d sensor_frame)
{
	if(link_name != _link_name)
	{
		throw std::invalid_argument("The link to which is attached the sensor should be the same as the link to which is attached the control frame in PosOriTask::setForceSensorFrame\n");
	}
	_T_link_sensor = sensor_frame;
	_T_control_sensor = _T_link_control.inverse() * _T_link_sensor;
}

void PosOriTask::updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
							    		    const Eigen::Vector3d sensed_moment_sensor_frame)
{
	// find the resolved sensed force and moment in control frame
	_sensed_force = _T_control_sensor.rotation() * sensed_force_sensor_frame;
	_sensed_moment = _T_control_sensor.translation().cross(_sensed_force) + _T_control_sensor.rotation() * sensed_moment_sensor_frame;

	// find the transform from base frame to control frame
	Eigen::Affine3d T_base_link;
	_robot->transform(T_base_link, _link_name);
	Eigen::Affine3d T_base_control = T_base_link * _T_link_control;

	// rotate the quantities in base frame
	_sensed_force = T_base_control.rotation() * _sensed_force;
	_sensed_moment = T_base_control.rotation() * _sensed_moment;
}

void PosOriTask::setForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;

	resetIntegratorsLinear();
}

void PosOriTask::updateForceAxis(const Eigen::Vector3d force_axis)
{
	Eigen::Vector3d normalized_axis = force_axis.normalized();

	_sigma_force = normalized_axis*normalized_axis.transpose();
	_sigma_position = Eigen::Matrix3d::Identity() - _sigma_force;
}

void PosOriTask::setLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;

	resetIntegratorsLinear();
}

void PosOriTask::updateLinearMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_position = normalized_axis*normalized_axis.transpose();
	_sigma_force = Eigen::Matrix3d::Identity() - _sigma_position;	
}

void PosOriTask::setFullForceControl()
{
	_sigma_force = Eigen::Matrix3d::Identity();
	_sigma_position.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setFullLinearMotionControl()
{
	_sigma_position = Eigen::Matrix3d::Identity();
	_sigma_force.setZero();

	resetIntegratorsLinear();
}

void PosOriTask::setMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;

	resetIntegratorsAngular();
}

void PosOriTask::updateMomentAxis(const Eigen::Vector3d moment_axis)
{
	Eigen::Vector3d normalized_axis = moment_axis.normalized();

	_sigma_moment = normalized_axis*normalized_axis.transpose();
	_sigma_orientation = Eigen::Matrix3d::Identity() - _sigma_moment;	
}

void PosOriTask::setAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;

	resetIntegratorsAngular();
}

void PosOriTask::updateAngularMotionAxis(const Eigen::Vector3d motion_axis)
{
	Eigen::Vector3d normalized_axis = motion_axis.normalized();

	_sigma_orientation = normalized_axis*normalized_axis.transpose();
	_sigma_moment = Eigen::Matrix3d::Identity() - _sigma_orientation;
}

void PosOriTask::setFullMomentControl()
{
	_sigma_moment = Eigen::Matrix3d::Identity();
	_sigma_orientation.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setFullAngularMotionControl()
{
	_sigma_orientation = Eigen::Matrix3d::Identity();
	_sigma_moment.setZero();

	resetIntegratorsAngular();
}

void PosOriTask::setClosedLoopForceControl()
{
	_closed_loop_force_control = true;
	resetIntegratorsLinear();
}

void PosOriTask::setOpenLoopForceControl()
{
	_closed_loop_force_control = false;
}

void PosOriTask::setClosedLoopMomentControl()
{
	_closed_loop_moment_control = true;
	resetIntegratorsAngular();
}

void PosOriTask::setOpenLoopMomentControl()
{
	_closed_loop_moment_control = false;
}

void PosOriTask::resetIntegrators()
{
	_integrated_orientation_error.setZero();
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
	_integrated_moment_error.setZero();
	_first_iteration = true;	
}

void PosOriTask::resetIntegratorsLinear()
{
	_integrated_position_error.setZero();
	_integrated_force_error.setZero();
}

void PosOriTask::resetIntegratorsAngular()
{
	_integrated_orientation_error.setZero();
	_integrated_moment_error.setZero();
}



