/*
 * TestTask.h
 *
 *      Author: Mikael Jorda
 */

#ifndef IIWA_FORCE_CONTROL_TEST_TASK_H_
#define IIWA_FORCE_CONTROL_TEST_TASK_H_

#include <Eigen/Dense>
#include <string>
#include <math.h>

class TestTask
{
public:

	TestTask(int dof)
	{
		link_name = "no link defined";
		pos_in_link = Eigen::Vector3d(0,0,0);

		current_position = Eigen::Vector3d::Zero();
		desired_position = Eigen::Vector3d::Zero();

		sensed_force = Eigen::Vector3d::Zero();
		desired_force = Eigen::Vector3d::Zero();
		force_integrated_error.setZero();
		force_feedback_control_signal.setZero();

		current_velocity = Eigen::Vector3d::Zero();
		desired_velocity = Eigen::Vector3d::Zero();
		saturated_velocity.setZero();
		velocity_saturation = false;

		kp = Eigen::Matrix3d::Zero();
		kv = Eigen::Matrix3d::Zero();
		kpf = Eigen::Matrix3d::Zero();
		kvf = Eigen::Matrix3d::Zero();
		kif = Eigen::Matrix3d::Zero();

		closed_loop_force_control = false;
		controller_frequency = 1000;

		passivity_enabled = false;
		Rc_ = 1.0;
		past_Rc_vector = Eigen::VectorXd::Zero(buffer_size);
		counter = 0;
		previous_Rc_ = 1.0;
	    PO_ = 0;
	    E_to_dissipate = 0;
	    current_power_input_ = 0;
	    current_power_output_ = 0;
	    release_counter_ = 333;
	    for(int i=0; i<5; i++)
	    {
	    	power_input_minimum_detection_buffer[i] = 0;
	    }

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

		// closed loop term
		if(closed_loop_force_control)
		{
			force_integrated_error += sigma_force * (sensed_force - desired_force)/controller_frequency;
			force_feedback_control_signal = -kpf * sigma_force * (sensed_force - desired_force) - kif * force_integrated_error;



			double tmp_Rc = Rc_;

			current_power_input_ = (double) (desired_force.transpose()*(sigma_force * force_feedback_control_signal));
	//        current_power_input_ *= tmp_Rc;
			// current_power_output_ = (double) (previous_force_related_forces_.transpose()*(sigma_force_*vel_));
			// current_power_output_ = 0.0;
			current_power_output_ = (double) (tmp_Rc*force_feedback_control_signal.transpose()*(sigma_force * force_feedback_control_signal));
			// std::cout << "power input : " << current_power_input_ - current_power_output_ << std::endl;

			if(passivity_enabled) // only resetting PO enabled
			{

			PO_ += (current_power_input_ - current_power_output_)/controller_frequency;

				for(int i=0; i<4; i++)
				{
					power_input_minimum_detection_buffer[i] = power_input_minimum_detection_buffer[i+1];
				}
				power_input_minimum_detection_buffer[4] = current_power_input_ - current_power_output_; // - power_adjustment;

				if(power_input_minimum_detection_buffer[0] > power_input_minimum_detection_buffer[1] &&
						power_input_minimum_detection_buffer[1] > power_input_minimum_detection_buffer[2] &&
						power_input_minimum_detection_buffer[2] < power_input_minimum_detection_buffer[3] &&
						power_input_minimum_detection_buffer[3] < power_input_minimum_detection_buffer[4] &&
						isPassive())
				{
					PO_ = power_input_minimum_detection_buffer[3] + power_input_minimum_detection_buffer[4];
				}


				// chose depending on PO and Vc. Not conclusive
				// if(!isPassive())
				// {
				// 	double Vc_square = force_feedback_control_signal.transpose() * sigma_force * force_feedback_control_signal;
				// 	if(Vc_square < 0.1)
				// 	{
				// 		Vc_square = 0;
				// 		Rc_ = 1;
				// 	}
				// 	else
				// 	{
				// 		Rc_ = 1+PO_*controller_frequency/Vc_square;
				// 		if(Rc_ < 0)
				// 		{
				// 			E_to_dissipate += -Vc_square/controller_frequency - PO_;
				// 			// std::cout << "\nRc negative. Still " << E_to_dissipate << " to dissipate" << std::endl;
				// 			std::cout << "\nRc negative" << std::endl;
				// 			std::cout << "PO :  " << PO_ << std::endl;
				// 			Rc_ = 0;
				// 		}
				// 	}
				// 	Rc_ = (Rc_ + previous_Rc_)/2;
				// 	PO_ += (1-Rc_)*Vc_square/controller_frequency;
				// 	std::cout << "Vc square : " << Vc_square << std::endl;
				// 	std::cout << "Rc : " << Rc_ << std::endl;
				// }
				// else
				// {
				// 	double Vc_square = force_feedback_control_signal.transpose() * sigma_force * force_feedback_control_signal;
				// 	Rc_ = 1.0;
				// 	Rc_ = (Rc_ + previous_Rc_)/2;
				// 	PO_ += (1-Rc_)*Vc_square/controller_frequency;
				// }
				// previous_Rc_ = Rc_;


				// if(counter % 10 == 0)
				// {
					double Vc_square = force_feedback_control_signal.transpose() * sigma_force * force_feedback_control_signal;
					Eigen::Vector3d deltaf = sensed_force - desired_force;
					double deltaf_square = deltaf.transpose()*sigma_force*deltaf;
					// deltaf_square = deltaf_square*deltaf_square;

					if(!isPassive())
					{
						// double increase_weight = (Rc_ - (1-atan(-PO_)))*(Rc_ - (1-atan(-PO_)));
						// Rc_ = (Rc_ + exp(PO_)*(Rc_ - exp(-PO_))*(Rc_ - exp(-PO_)))/(1+(Rc_ - exp(-PO_))*(Rc_ - exp(-PO_)));
						// Rc_ = exp(-PO_);
						// Rc_ = 1-atan(-PO_);
						Rc_ = (Rc_ + (1-2*atan(-0.1*PO_)/M_PI)*deltaf_square) / (1+deltaf_square);
						// Rc_ -= atan(-PO_)/M_PI/10;
						// Rc_ = (Rc_ + exp(PO_)*deltaf_square) / (1+deltaf_square);
						// Rc_ = (Rc_ + (1-atan(-PO_))*(Rc_ - (1-atan(-PO_)))*(Rc_ - (1-atan(-PO_))))/(1+(Rc_ - (1-atan(-PO_)))*(Rc_ - (1-atan(-PO_))));
						// Rc_ = (exp(PO_) + past_Rc_vector.sum())/(buffer_size+1);

						// Eigen::VectorXd tmp = past_Rc_vector.head(buffer_size-1);
						// past_Rc_vector.tail(buffer_size-1) = tmp;
						// past_Rc_vector(0) = exp(PO_);
						// Rc_ = (2*Rc_ + previous_Rc_)/3;
						if(Rc_ < 0)
						{
							Rc_ = 0;
						}
						std::cout << "Rc : " << Rc_ << std::endl;
					}
					else
					{
						// double increase_weight = (Rc_ - 1)*(Rc_ - 1);
						Rc_ = (Rc_ + 1.0*deltaf_square)/(1+deltaf_square);
						// Rc_ += 0.05;
						// Rc_ = (Rc_ + 1*(Rc_ - 1)*(Rc_ - 1))/(1+(Rc_ - 1)*(Rc_ - 1));
						// Rc_ = (Rc_ + 1*abs(Rc_ - exp(-PO_)))/(1+abs(Rc_ - exp(-PO_)));
						// Rc_ = 1;
						// Rc_ = (1 + past_Rc_vector.sum())/(buffer_size+1);

						// Eigen::VectorXd tmp = past_Rc_vector.head(buffer_size-1);
						// past_Rc_vector.tail(buffer_size-1) = tmp;
						// past_Rc_vector(0) = 1;
						// Rc_ = (10 + past_Rc_vector.sum())/30;
						if(Rc_ > 1)
						{
							Rc_ = 1;
						}
					}
					PO_ += (tmp_Rc-Rc_)*Vc_square/controller_frequency;
				// }
				// Rc_ = 0.9*Rc_;


				// std::cout << past_Rc_vector.transpose() << std::endl;
				// previous_Rc_ = Rc_;

				// Rc_ = 0.40;

				// very heuristic passivity controller tuned for the kuka
				// if(!isPassive() && release_counter_ < 200)
				// {
				// 	Rc_ /= 2;
				// 	release_counter_ = 333;
				// }
				// else if(release_counter_ <= 0)
				// {
				// 	Rc_ += 0.1;
				// 	release_counter_ = 333;
				// }
				// else
				// {
				// 	release_counter_--;
				// }

				// if(release_counter_ < 0)
				// {
				// 	release_counter_ = 0;
				// }

				// if(Rc_ < 0.1)
				// {
				// 	Rc_ = 0.1;
				// }
				// if(Rc_ > 1)
				// {
				// 	Rc_ = 1;
				// }
				counter++;
			}
			else
			{
				Rc_ = 1.0;
			}

			double eff_mass_f = force_axis.transpose() * Lambda * force_axis;
			// std::cout << "effective mass : " << eff_mass_f << std::endl;

			// force_related_forces = desired_force + sigma_force * (Rc_ * force_feedback_control_signal - kvf * current_velocity);
			force_related_forces = Lambda * sigma_force / eff_mass_f * (desired_force + (Rc_ * force_feedback_control_signal - kvf * current_velocity));
			// force_related_forces = sigma_force * desired_force + Lambda * sigma_force * (Rc_ * force_feedback_control_signal - kvf * current_velocity);
			// force_related_forces = Lambda * sigma_force * Rc_ * force_feedback_control_signal - kvf * current_velocity;
		}
		else
		{
			// open loop force (feedforward term)
			force_related_forces = sigma_force * desired_force;
		}

		// PD controller for position part
		if(velocity_saturation)
		{
			// assumes the gains are diagonal matrices
			desired_velocity = kp * kv.inverse() * (desired_position - current_position);
			for(int i=0; i<3; i++)
			{
				if(desired_velocity(i) > saturated_velocity(i))
				{
					desired_velocity(i) = saturated_velocity(i);
				}
				else if(desired_velocity(i) < -saturated_velocity(i))
				{
					desired_velocity(i) = -saturated_velocity(i);
				}
			}
			position_related_forces = Lambda * sigma_position * (-kv*(current_velocity - desired_velocity));
		}
		else
		{
			position_related_forces = Lambda * (sigma_position * (-kp*(current_position - desired_position) - kv*(current_velocity - desired_velocity)));
		}

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

	void useVelocitySaturation(const bool b, const Eigen::Vector3d& sat = Eigen::Vector3d::Zero())
	{
		velocity_saturation = b;
		saturated_velocity = sat;
		for(int i=0; i<3; i++)
		{
			if(sat(i) < 0)
			{
				std::cout << "WARNING : saturation velocity " << i << " should be positive. Set to zero" << std::endl;
				saturated_velocity(i) = 0;
			}
		}
	}

	void enablePassivity()
	{
		passivity_enabled = true;
		Rc_ = 1.0;
		previous_Rc_ = 0;
		past_Rc_vector.setZero();
		counter = 0;
	    PO_ = 0;
	    current_power_input_ = 0;
	    current_power_output_ = 0;
	    release_counter_ = 333;
	    for(int i=0; i<5; i++)
	    {
	    	power_input_minimum_detection_buffer[i] = 0;
	    }
	}

	bool isPassive()
	{
	    if(!passivity_enabled)
	    {
	        std::cout << "Passivity not enabled\n";
	        return true;
	    }
	    return (PO_ > 0);
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

	void setKpf(const double d)
	{
		kpf = d*Eigen::Matrix3d::Identity();
	}

	void setKvf(const double d)
	{
		kvf = d*Eigen::Matrix3d::Identity();
	}

	void setKif(const double d)
	{
		kif = d*Eigen::Matrix3d::Identity();
	}

	std::string link_name;
	Eigen::Vector3d pos_in_link;

	Eigen::Vector3d current_position;
	Eigen::Vector3d desired_position;

	Eigen::Vector3d current_velocity;
	Eigen::Vector3d desired_velocity;

	Eigen::Vector3d sensed_force;
	Eigen::Vector3d desired_force;

	Eigen::Vector3d force_integrated_error;
	double controller_frequency;
	Eigen::Vector3d force_feedback_control_signal;

	bool passivity_enabled = false;
	double Rc_ = 1.0;
	double previous_Rc_ = 1.0;
	Eigen::VectorXd past_Rc_vector;
	unsigned long long counter = 0;
	int buffer_size = 500;
    double PO_;
    double E_to_dissipate;
    double current_power_input_;
    double current_power_output_;
    int release_counter_;
    double power_input_minimum_detection_buffer[5] = {0,0,0,0,0};

	Eigen::Matrix3d kp;
	Eigen::Matrix3d kv;

	Eigen::Matrix3d kpf;
	Eigen::Matrix3d kvf;
	Eigen::Matrix3d kif;

	Eigen::MatrixXd jacobian;
	Eigen::MatrixXd projected_jacobian;
	Eigen::MatrixXd Lambda;
	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd N;

// private:
	Eigen::Vector3d task_force;

	Eigen::Vector3d position_related_forces;
	Eigen::Vector3d force_related_forces;

	Eigen::Vector3d force_axis;
	Eigen::Matrix3d sigma_force;
	Eigen::Matrix3d sigma_position;

	bool closed_loop_force_control = false;
	bool velocity_saturation = false;
	Eigen::Vector3d saturated_velocity;
};

#endif