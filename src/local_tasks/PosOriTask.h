/*
 * PosOriTask.h
 *
 *      This class creates a 6Dof position + orientation hybrid controller for a robotic manipulator using operational space formulation and an underlying PID compensator.
 *      If used for hybrid position force control, assumes a force sensor is attached to the same link as the control frame and the force sensed values are given in sensor frame.
 *      Besides, the force sensed and moment sensed are assumed to be the force and moment that the robot applies to the environment.
 *      It requires a robot model parsed from a urdf file to a Sai2Model object, as well as the definition of a control frame
 *      as a link at which the frame is attached, and an affine transform that determines the position and orientation of the control frame in this link
 *
 *      Author: Mikael Jorda
 */

#ifndef IIWAFORCECONTROL_POSORI_TASK_H_
#define IIWAFORCECONTROL_POSORI_TASK_H_

#include "Sai2Model.h"
#include <Eigen/Dense>
#include <string>
#include <chrono>

class PosOriTask
{
public:

	//------------------------------------------------
	// Constructors
	//------------------------------------------------

	/**
	 * @brief Constructor that takes an Affine3d matrix for definition of the control frame. Creates a full position controller by default.
	 * 
	 * @param robot           A pointer to a Sai2Model object for the robot that is to be controlled	
	 * @param link_name       The name of the link in the robot at which to attach the control frame
	 * @param control_frame The position and orientation of the control frame in local link coordinates
	 * @param sensor_frame  The position and orientation of the sensor frame in local link coordinates
	 */
	PosOriTask(Sai2Model::Sai2Model* robot, 
		            const std::string link_name, 
		            const Eigen::Affine3d control_frame = Eigen::Affine3d::Identity(),
		            const Eigen::Affine3d sensor_frame = Eigen::Affine3d::Identity());

	//------------------------------------------------
	// Methods
	//------------------------------------------------

	// -------- core methods --------

	/**
	 * @brief update the task model (jacobians, task inertia and nullspace matrices)
	 * @details This function updates the jacobian, projected jacobian, task inertia matrix (Lambda), dynamically consistent inverse of the Jacobian (Jbar)
	 * and nullspace matrix of the task N. This function uses the robot model and assumes it has been updated. 
	 * There is no use to calling it if the robot kinematics or dynamics have not been updated since the last call.
	 * This function takes the N_prec matrix as a parameter which is the product of the nullspace matrices of the higher priority tasks.
	 * The N matrix will be the matrix to use as N_prec for the subsequent tasks.
	 * In order to get the nullspace matrix of this task alone, one needs to compute _N * _N_prec.transpose().	
	 * 
	 * @param N_prec The nullspace matrix of all the higher priority tasks. If this is the highest priority task, use identity of size n*n where n in the number of DoF of the robot.
	 */
	void updateTaskModel(const Eigen::MatrixXd N_prec);

	/**
	 * @brief Computes the torques associated with this task.
	 * @details Computes the torques taking into account the last model update and updated values for the robot joint positions/velocities
	 * assumes the desired orientation and angular velocity has been updated
	 * 
	 * @param task_joint_torques the vector to be filled with the new joint torques to apply for the task
	 */
	void computeTorques(Eigen::VectorXd& task_joint_torques);

	// -------- force control related methods --------

	void setForceSensorFrame(const std::string link_name, const Eigen::Affine3d sensor_frame);

	/**
	 * @brief Updates the velues of the sensed force and sensed moment from sensor values
	 * @details Assumes that the sensor is attached to the same link as the control frame and that the member variable _T_control_sensor has been set as the 
	 * transformation matrix from the control frame to the sensor frame with the function setSensorFrame.
	 * The force and moment values given to this function are assumed to be in the force sensor frame (values taken directly from the force sensor)
	 * These values are supposed to be the forces that the sensor applies to the environment (so the opposite of what the sensor feels)
	 * 
	 * @param sensed_force_sensor_frame The sensed force as the force that the sensor applies to the environment in sensor frame
	 * @param sensed_moment_sensor_frame The sensed moment as the moment that the sensor applies to the environment in sensor frame
	 */
	void updateSensedForceAndMoment(const Eigen::Vector3d sensed_force_sensor_frame, 
								    const Eigen::Vector3d sensed_moment_sensor_frame);

	/**
	 * @brief Sets the force controlled axis for a hybrid position force controller with 1 DoF force and 2 DoF motion
	 * @details This is the function to use in order to get the position part of the controller behave as a Hybrid Force/Motion controller with 1 Dof force.
	 * The motion is controlled orthogonally to the force.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a force controller.
	 */
	void setForceAxis(const Eigen::Vector3d force_axis);

	/**
	 * @brief Updates the force controlled axis for a hybrid position force controller with 1 DoF force and 2 DoF motion
	 * @details Use this function in situations when the force axis needs to be updated (as an estimated normal for example) over time.
	 * This does not reset the integral terms.
	 * In setting up the controller for the first time, prefer setForceAxis.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a force controller.
	 */
	void updateForceAxis(const Eigen::Vector3d force_axis);

	/**
	 * @brief Sets the motion controlled axis for a hybrid position force controller with 2 DoF force and 1 DoF motion
	 * @details This is the function to use in order to get the position part of the controller behave as a Hybrid Force/Motion controller with 2 Dof force.
	 * The motion is controlled along one axis and the force is controlled orthogonally to the motion
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a motion controller.
	 */
	void setLinearMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief Sets the motion controlled axis for a hybrid position force controller with 2 DoF force and 1 DoF motion
	 * @details Use this function in situations when the motion axis needs to be updated over time.
	 * This does not reset the integral terms.
	 * In setting up the controller for the first time, prefer setMotionAxis.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a motion controller.
	 */
	void updateLinearMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief Sets the linear part of the task as a full 3DoF force controller
	 * @details This is the function to use in order to get the position part of the controller behave as pure Force controller with 3 Dof force.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 */
	void setFullForceControl();

	/**
	 * @brief Sets the linear part of the task as a full 3DoF motion controller
	 * @details This is the function to use in order to get the position part of the controller behave as pure Motion controller with 3 Dof linear motion.
	 * It is de default behavior of the controller.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 */
	void setFullLinearMotionControl();

	/**
	 * @brief Sets the moment controlled axis for a hybrid orientation moment controller with 1 DoF moment and 2 DoF orientation
	 * @details This is the function to use in order to get the orientation part of the controller behave as a Hybrid Force/Motion controller with 1 Dof moment.
	 * The rotational motion is controlled orthogonally to the moment.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 * 
	 * @param moment_axis         The axis in control frame coordinates along which the controller behaves as a moment controller.
	 */
	void setMomentAxis(const Eigen::Vector3d moment_axis);

	/**
	 * @brief Sets the moment controlled axis for a hybrid orientation moment controller with 1 DoF moment and 2 DoF orientation
	 * @details Use this function in situations when the moment axis needs to be updated over time.
	 * This does not reset the integral terms.
	 * In setting up the controller for the first time, prefer setMomentAxis.
	 * 
	 * @param moment_axis         The axis in control frame coordinates along which the controller behaves as a moment controller.
	 */
	void updateMomentAxis(const Eigen::Vector3d moment_axis);

	/**
	 * @brief Sets the angular movement controlled axis for a hybrid orientation moment controller with 2 DoF moment and 1 DoF motion
	 * @details This is the function to use in order to get the orientation part of the controller behave as a Hybrid Force/Motion controller with 2 Dof moment.
	 * The rotational motion is controlled along one axis and the moment is controlled orthogonally to the motion
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a rotational motion controller.
	 */
	void setAngularMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief Sets the angular movement controlled axis for a hybrid orientation moment controller with 2 DoF moment and 1 DoF motion
	 * @details Use this function in situations when the angular motion axis needs to be updated over time.
	 * This does not reset the integral terms.
	 * In setting up the controller for the first time, prefer setAngularMotionAxis.
	 * 
	 * @param force_axis         The axis in control frame coordinates along which the controller behaves as a rotational motion controller.
	 */
	void updateAngularMotionAxis(const Eigen::Vector3d motion_axis);

	/**
	 * @brief Sets the angular part of the task as a full 3DoF moment controller
	 * @details This is the function to use in order to get the orientation part of the controller behave as pure moment controller with 3 Dof moment.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 */
	void setFullMomentControl();

	/**
	 * @brief Sets the angular part of the task as a full 3DoF motion controller
	 * @details This is the function to use in order to get the orientation part of the controller behave as pure Motion controller with 3 Dof angular motion.
	 * It is de default behavior of the controller.
	 * It can be called anytime to change the behavior of the controller and reset the integral terms.
	 */
	void setFullAngularMotionControl();

	/**
	 * @brief Changes the behavior to closed loop force control for the force controlled directions in the linear part of the controller
	 */
	void setClosedLoopForceControl();

	/**
	 * @brief Changes the behavior to open loop force control for the force controlled directions in the linear part of the controller
	 * (default behavior)
	 */
	void setOpenLoopForceControl();

	/**
	 * @brief Changes the behavior to closed loop moment control for the moment controlled directions in the angular part of the controller
	 */
	void setClosedLoopMomentControl();

	/**
	 * @brief Changes the behavior to open loop moment control for the moment controlled directions in the angular part of the controller
	 */
	void setOpenLoopMomentControl();
	// ------- helper methods -------

	/**
	 * @brief Resets all the integrated errors used in I terms
	 */
	void resetIntegrators();

	/**
	 * @brief Resets the integrated errors used in I terms for linear part of task (position_integrated_error and force_integrated_error)
	 */
	void resetIntegratorsLinear();

	/**
	 * @brief Resets the integrated errors used in I terms for angular part of task (orientation_integrated_error and moment_integrated_error)
	 */
	void resetIntegratorsAngular();

	void enablePassivity();
	void disablePassivity();

	void enableForcePassivity();
	void disableForcePassivity();

	void enableMomentPassivity();
	void disableMomentPassivity();

	//------------------------------------------------
	// Attributes
	//------------------------------------------------

	Sai2Model::Sai2Model* _robot;
	std::string _link_name;
	Eigen::Affine3d _T_link_control;   // in link_frame

	// movement quantities
	Eigen::Vector3d _current_position;      // robot frame
	Eigen::Vector3d _desired_position;      // robot frame
	Eigen::Matrix3d _current_orientation;   // robot frame
	Eigen::Matrix3d _desired_orientation;   // robot frame

	Eigen::Vector3d _current_linear_velocity;           // robot frame
	Eigen::Vector3d _desired_linear_velocity;           // robot frame
	Eigen::Vector3d _current_angular_velocity;   // robot frame
	Eigen::Vector3d _desired_angular_velocity;   // robot frame

	double _kp_pos, _kp_ori;
	double _kv_pos, _kv_ori;
	double _ki_pos, _ki_ori;

	Eigen::Vector3d _orientation_error;               // robot frame
	Eigen::Vector3d _integrated_orientation_error;    // robot frame
	Eigen::Vector3d _integrated_position_error;       // robot frame
	
	Eigen::Matrix3d _sigma_position;        // control frame
	Eigen::Matrix3d _sigma_orientation;     // control frame

	// force quantities
	Eigen::Affine3d _T_link_sensor; 
	Eigen::Affine3d _T_control_sensor;

	Eigen::Vector3d _desired_force;   // robot frame
	Eigen::Vector3d _sensed_force;    // robot frame
	Eigen::Vector3d _desired_moment;  // robot frame
	Eigen::Vector3d _sensed_moment;   // robot frame

	double _kp_force, _kp_moment;
	double _kv_force, _kv_moment;
	double _ki_force, _ki_moment;

	Eigen::Vector3d _integrated_force_error;    // robot frame
	Eigen::Vector3d _integrated_moment_error;   // robot frame

	Eigen::Matrix3d _sigma_force;     // control frame
	Eigen::Matrix3d _sigma_moment;    // control frame

	bool _closed_loop_force_control;
	bool _closed_loop_moment_control;

	// passivity related things
	bool _force_passivity_enabled;
	bool _moment_passivity_enabled;
	double _Rc_force;
	double _Rc_moment;
	double _PO_force;
	double _PO_moment;

	Eigen::VectorXd _PO_buffer_force;
	Eigen::VectorXd _PO_buffer_moment;

	const int _PC_max_counter = 10;
	int _PC_force_counter = _PC_max_counter;
	int _PC_moment_counter = _PC_max_counter;

	// task force (6D vector of forces and moments at control frame)
	Eigen::VectorXd _task_force;   // robot frame

	// model quantities
	Eigen::MatrixXd _jacobian;
	Eigen::MatrixXd _projected_jacobian;
	Eigen::MatrixXd _Lambda;
	Eigen::MatrixXd _Jbar;
	Eigen::MatrixXd _N;
	Eigen::MatrixXd _N_prec;

	// timing for I term
	std::chrono::high_resolution_clock::time_point _t_prev;
	std::chrono::high_resolution_clock::time_point _t_curr;
	std::chrono::duration<double> _t_diff;
	bool _first_iteration;
};


/* IIWAFORCECONTROL_POSORI_TASK_H_ */
#endif