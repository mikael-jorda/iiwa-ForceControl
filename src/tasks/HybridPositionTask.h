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
	    PO_input_ = 0;
	    PO_output_ = 0;
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
			force_integrated_error += (sensed_force - desired_force)/controller_frequency;
			force_feedback_control_signal = -kpf * (sensed_force - desired_force) - kif * force_integrated_error;



			double tmp_Rc = Rc_;

			current_power_input_ = (double) (desired_force.transpose()*(sigma_force * force_feedback_control_signal));
	//        current_power_input_ *= tmp_Rc;
			// current_power_output_ = (double) (previous_force_related_forces_.transpose()*(sigma_force_*vel_));
			current_power_output_ = 0.0;

			if(passivity_enabled) // only resetting PO enabled
			{

			PO_input_ += current_power_input_; // add latest value
			PO_output_ += current_power_output_; // 

				for(int i=0; i<4; i++)
				{
					power_input_minimum_detection_buffer[i] = power_input_minimum_detection_buffer[i+1];
				}
				power_input_minimum_detection_buffer[4] = current_power_input_; // - power_adjustment;

				if(power_input_minimum_detection_buffer[0] > power_input_minimum_detection_buffer[1] &&
						power_input_minimum_detection_buffer[1] > power_input_minimum_detection_buffer[2] &&
						power_input_minimum_detection_buffer[2] < power_input_minimum_detection_buffer[3] &&
						power_input_minimum_detection_buffer[3] < power_input_minimum_detection_buffer[4] &&
						isPassive())
				{
					PO_input_ = power_input_minimum_detection_buffer[3] + power_input_minimum_detection_buffer[4];
				}


				// very heuristic passivity controller tuned for the kuka
				if(!isPassive() && release_counter_ < 200)
				{
					Rc_ /= 2;
					release_counter_ = 333;
				}
				else if(release_counter_ <= 0)
				{
					Rc_ += 0.1;
					release_counter_ = 333;
				}
				else
				{
					release_counter_--;
				}

				if(release_counter_ < 0)
				{
					release_counter_ = 0;
				}

				if(Rc_ < 0.1)
				{
					Rc_ = 0.1;
				}
				if(Rc_ > 1)
				{
					Rc_ = 1;
				}
			}
			else
			{
				Rc_ = 1.0;
			}

			force_related_forces = sigma_force * desired_force + sigma_force * Rc_ * force_feedback_control_signal - kvf * current_velocity;
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
	    PO_input_ = 0;
	    PO_output_ = 0;
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
	    return (PO_input_-PO_output_ > 1);
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
    double PO_input_;
    double PO_output_;
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