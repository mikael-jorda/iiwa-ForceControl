#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "model/ModelInterface.h"
#include "graphics/ChaiGraphics.h"
#include "simulation/Sai2Simulation.h"
#include <dynamics3d.h>

#include "force_sensor/ForceSensorSim.h"
#include "timer/LoopTimer.h"
#include "tasks/BasePrimitive.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

#define INITIAL                0
#define GO_TO_CONTACT          1
#define ALIGN                  2

int state = INITIAL;

using namespace std;

const string world_file = "../resources/00-test_base_primitive/world.urdf";
const string robot_file = "../resources/00-test_base_primitive/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";

const string camera_name = "camera";

// simulation loop
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;



int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_file << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Graphics::ChaiGraphics(world_file, Graphics::urdf, false);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	// load robots
	auto robot = new Model::ModelInterface(robot_file, Model::rbdl, Model::urdf, false);
	robot->updateModel();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, Simulation::urdf, false);
	sim->setCollisionRestitution(0);
	sim->setCoeffFrictionStatic(0.2);

	// set initial condition
	robot->_q <<  90.0/180.0*M_PI,
				 -30.0/180.0*M_PI,
				  00.0/180.0*M_PI,
				  60.0/180.0*M_PI,
				 -08.0/180.0*M_PI,
				 -90.0/180.0*M_PI,
				  00.0/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

	double last_cursorx, last_cursory;

    // set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();

		// move scene camera as required
    	// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
    	Eigen::Vector3d cam_depth_axis;
    	cam_depth_axis = camera_lookat - camera_pos;
    	cam_depth_axis.normalize();
    	Eigen::Vector3d cam_up_axis;
    	// cam_up_axis = camera_vertical;
    	// cam_up_axis.normalize();
    	cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
	    Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
    	cam_roll_axis.normalize();
    	Eigen::Vector3d cam_lookat_axis = camera_lookat;
    	cam_lookat_axis.normalize();
    	if (fTransXp) {
	    	camera_pos = camera_pos + 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_roll_axis;
	    }
	    if (fTransXn) {
	    	camera_pos = camera_pos - 0.05*cam_roll_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_roll_axis;
	    }
	    if (fTransYp) {
	    	// camera_pos = camera_pos + 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos + 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat + 0.05*cam_up_axis;
	    }
	    if (fTransYn) {
	    	// camera_pos = camera_pos - 0.05*cam_lookat_axis;
	    	camera_pos = camera_pos - 0.05*cam_up_axis;
	    	camera_lookat = camera_lookat - 0.05*cam_up_axis;
	    }
	    if (fTransZp) {
	    	camera_pos = camera_pos + 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat + 0.1*cam_depth_axis;
	    }	    
	    if (fTransZn) {
	    	camera_pos = camera_pos - 0.1*cam_depth_axis;
	    	camera_lookat = camera_lookat - 0.1*cam_depth_axis;
	    }
	    if (fRotPanTilt) {
	    	// get current cursor position
	    	double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
	    }
	    graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
	    glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void control(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	robot->updateModel();

	int dof = robot->dof();
	Eigen::VectorXd command_torques = Eigen::VectorXd::Zero(dof);
	Eigen::MatrixXd N_prec = Eigen::MatrixXd::Zero(dof,dof);

	// joint task
	Eigen::VectorXd joint_task_torques = Eigen::VectorXd::Zero(dof);
	double kp_joint = 0.0;
	double kv_joint = 10.0;

	string link_name = "link6";
	Eigen::Vector3d control_point = Eigen::Vector3d(0.0,0.0,0.05);

	// base primitive
	BasePrimitive base_primitive = BasePrimitive(dof);

	base_primitive.setKpPos(100.0);
	base_primitive.setKvPos(20.0);
	base_primitive.setKvOri(600.0);
	base_primitive.setKvOri(50.0);

	Eigen::Vector3d initial_position;
	robot->position(initial_position, link_name, control_point);
	base_primitive.current_position = initial_position;
	base_primitive.desired_position = initial_position;
	Eigen::Matrix3d initial_orientation;
	robot->rotation(initial_orientation, link_name);
	base_primitive.current_orientation = initial_orientation;
	base_primitive.desired_orientation = initial_orientation;

	Eigen::VectorXd base_primitive_torques = Eigen::VectorXd::Zero(dof);

	// force sensor
	Eigen::Vector3d sensed_forces;
	Eigen::Vector3d sensed_moments;

	string sensor_link_name = "link6";
	Eigen::Affine3d fsensor_location = Eigen::Affine3d::Identity();
	fsensor_location.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
	auto fsensor = new ForceSensorSim(robot_name, sensor_link_name, fsensor_location, robot);

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long controller_counter = 0;

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		// read force sensor values
		fsensor->update(sim);
		fsensor->getForce(sensed_forces);
		sensed_forces = -sensed_forces;
		fsensor->getMoment(sensed_moments);
		sensed_moments = -sensed_moments;
		base_primitive.sensed_force = sensed_forces;
		base_primitive.sensed_moment = sensed_moments;

		// update tasks model
		N_prec = Eigen::MatrixXd::Identity(dof,dof);

		// orientation controller
		robot->J_0(base_primitive.jacobian, link_name, control_point);
		base_primitive.projected_jacobian = base_primitive.jacobian * N_prec;
		robot->operationalSpaceMatrices(base_primitive.Lambda, base_primitive.Jbar, base_primitive.N,
								base_primitive.projected_jacobian, N_prec);
		N_prec = base_primitive.N;

		// -------------------------------------------
		////////////////////////////// Compute joint torques
		double time = controller_counter/control_freq;

		//orientation task
		robot->rotation(base_primitive.current_orientation, link_name);
		base_primitive.current_angular_velocity = base_primitive.projected_jacobian.block(3,0,3,dof)*robot->_dq;

		// -- pos task
		robot->position(base_primitive.current_position, link_name, control_point);
		base_primitive.current_velocity = base_primitive.projected_jacobian.block(0,0,3,dof) * robot->_dq;

		if(state == INITIAL)
		{
			if(time > 1.0)
			{
				state = GO_TO_CONTACT;
			}
		}

		if(state == GO_TO_CONTACT)
		{
			base_primitive.desired_position(2) -= 0.0001;
			if(sensed_forces(2) < -5)
			{
				base_primitive.desired_moment = Eigen::Vector3d(0.0,0.0,0.0);
				base_primitive.setKpfOri(10.0);
				base_primitive.setKvfOri(20.0);
				base_primitive.setAngularForceControl();

				base_primitive.desired_force = Eigen::Vector3d(0.0,0.0,-5);

				state = ALIGN;
			}
			if(controller_counter % 1000 == 0)
			{
				cout << "sensed forces : " << sensed_forces.transpose() << endl << "sensed moments : " << sensed_moments.transpose() << endl << endl;
			}
		}

		if(state == ALIGN)
		{
			Eigen::Vector3d z_axis = base_primitive.current_orientation.block(0,2,3,1);
			base_primitive.setLinearHybridType(HybridType::single_force_axis, z_axis);
			base_primitive.desired_force = 5.0 * z_axis;

			if(controller_counter % 1000 == 0)
			{
				cout << "sensed forces : " << sensed_forces.transpose() << endl << "sensed moments : " << sensed_moments.transpose() << endl;
				cout << "z axis : " << z_axis.transpose() << endl << endl;
			}
		}

		base_primitive.computeTorques(base_primitive_torques);

		// joint task
		joint_task_torques = N_prec.transpose() * robot->_M *(-kv_joint*robot->_dq);

		//------ Final torques
		command_torques = base_primitive_torques + joint_task_torques;
		// command_torques.setZero();

		// -------------------------------------------
		sim->setJointTorques(robot_name, command_torques);
		

		// -------------------------------------------
		if(controller_counter % 500 == 0)
		{
			// cout << time << endl;
			// cout << command_torques.transpose() << endl;
			// cout << 180.0/M_PI*robot->_q.transpose() << endl;
			// cout << endl;
			// cout << controller_counter << endl;
		}

		controller_counter++;

		// -------------------------------------------
		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

}

//------------------------------------------------------------------------------
void simulation(Model::ModelInterface* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;



	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(2000); 
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
    std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}


//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
		/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
    switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
    }
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
		switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}