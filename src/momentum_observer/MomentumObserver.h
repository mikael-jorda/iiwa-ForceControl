/*
 * MomentumObserver.h
 *
 *	A class to compute the generalized momentum observer for collision detection as described in 
 * "Haddadin et al : Robot Collisions: A Survey on Detection, Isolation, and Identification, TRO 2017"
 *	
 *
 *  Created on: Feb 28, 2018
 *      Author: Mikael Jorda
 */

#ifndef MOMENTUMOBSERVER_H_
#define MOMENTUMOBSERVER_H_

#include "Sai2Model.h"


class MomentumObserver
{
public:

	/**
	 * @brief Constructor that takes the integration step as a parameter
	 * 
	 * @param dt   Integration timestep
	 */
	MomentumObserver(Sai2Model::Sai2Model* robot, 
					const double dt,
					const bool built_in_grav_comp = true)
	{
		_robot = robot;
		_dt = dt;
		_built_in_grav_comp = built_in_grav_comp;

		_previous_r_out.setZero(_robot->dof());
		_integrated_momentum_estimate.setZero(_robot->dof());

		_K0 = 100.0;

		std::cout << "dt : " << _dt << std::endl;
		std::cout << "K0 : " << _K0 << std::endl;
	}


	~MomentumObserver(){}


	void update(Eigen::VectorXd& r_out,
			const Eigen::VectorXd command_torques)
	{
		Eigen::VectorXd beta = Eigen::VectorXd::Zero(_robot->dof());
		Eigen::VectorXd grav = Eigen::VectorXd::Zero(_robot->dof());
		Eigen::MatrixXd C =Eigen::MatrixXd::Zero(_robot->dof(),_robot->dof());

		if(!_built_in_grav_comp)
		{
			_robot->gravityVector(grav);
		}
		_robot->factorizedChristoffelMatrix(C);
		beta = grav - C.transpose()*_robot->_dq;

		Eigen::VectorXd p = _robot->_M * _robot->_dq;

		_integrated_momentum_estimate += (command_torques - beta + _previous_r_out)*_dt;

		r_out = _K0*(p - _integrated_momentum_estimate);

		_previous_r_out = r_out;
	}





public:

	Sai2Model::Sai2Model* _robot;

	double _dt;
	double _K0;

	bool _built_in_grav_comp;

	Eigen::VectorXd _previous_r_out;
	Eigen::VectorXd _integrated_momentum_estimate;

};

#endif