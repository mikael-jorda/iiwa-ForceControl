#!/usr/bin/env python

# read redis keys and dump them to a file
import redis, time, signal, sys
import os

runloop = True

# handle ctrl-C and close the files
def signal_handler(signal, frame):
	global runloop
	runloop = False
	print('  Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'test1'
if not os.path.exists(folder):
    os.makedirs(folder)

header = ''

file_cartesian_position = open(folder + '/' + header + 'cartesian_positions.txt','w')
file_orientation = open(folder + '/' + header + 'orientation.txt','w')
file_joint_positions = open(folder + '/' + header + 'joint_positions.txt','w')
file_joint_velocities = open(folder + '/' + header + 'joint_velocities.txt','w')
file_commanded_torques = open(folder + '/' + header + 'commanded_torques.txt','w')

file_joint_positions.write('Timestamp \t Joint positions\n')
file_joint_velocities.write('Timestamp \t Joint velocities\n')
file_commanded_torques.write('Timestamp \t Commanded joint torques\n')
file_cartesian_position.write('Timestamp \t desired position \t position\n')
file_orientation.write('Timestamp \t desired orientation \t orientation\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
JOINT_ANGLES_DES_KEY  = "scl::robot::iiwaBot::sensors::q_des";
JOINT_TORQUES_COMMANDED_KEY = "scl::robot::iiwaBot::actuators::fgc";

JOINT_ANGLES_KEY  = "scl::robot::iiwaBot::sensors::q";
JOINT_VELOCITIES_KEY = "scl::robot::iiwaBot::sensors::dq";
FGC_ENABLE_KEY  = "scl::robot::iiwaBot::fgc_command_enabled";
JOINT_TORQUES_KEY = "scl::robot::iiwaBot::sensors::fgc";
SIM_TIMESTAMP_KEY = "scl::robot::iiwaBot::timestamp";

POS_TASK_X_DESIRED_KEY = "scl::robot::iiwaBot::pos_task_x_desired";
POS_TASK_X_KEY = "scl::robot::iiwaBot::pos_task_x";
ORI_TASK_R_DESIRED_KEY = "scl::robot::iiwaBot::ori_task_R_desired";
ORI_TASK_R_KEY = "scl::robot::iiwaBot::ori_task_R";

# data logging frequency
logger_frequency = 100.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print 'Start Logging Data\n'

while(runloop):
	t += logger_period

	file_joint_positions.write(r_server.get(SIM_TIMESTAMP_KEY) + '\t' + r_server.get(JOINT_ANGLES_KEY) + '\n')
	file_joint_velocities.write(r_server.get(SIM_TIMESTAMP_KEY) + '\t' + r_server.get(JOINT_VELOCITIES_KEY) + '\n')
	file_commanded_torques.write(r_server.get(SIM_TIMESTAMP_KEY) + '\t' + r_server.get(JOINT_TORQUES_COMMANDED_KEY) + '\n')
	file_cartesian_position.write(r_server.get(SIM_TIMESTAMP_KEY) + '\t' + r_server.get(POS_TASK_X_DESIRED_KEY) + '\t' + r_server.get(POS_TASK_X_KEY) + '\n')
	file_orientation.write(r_server.get(SIM_TIMESTAMP_KEY) + '\t' + r_server.get(ORI_TASK_R_DESIRED_KEY) + '\t' + r_server.get(ORI_TASK_R_KEY) + '\n')

	time.sleep(max(0.0,t-time.time()))

file_joint_positions.close()
file_joint_velocities.close()
file_commanded_torques.close()
