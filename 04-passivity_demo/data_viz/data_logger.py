#!/usr/bin/env python

# read redis keys and dump them to a file
import redis, time, signal, sys
import os
import json

runloop = True
counter = 0

# handle ctrl-C and close the files
def signal_handler(signal, frame):
	global runloop
	runloop = False
	print('  Exiting data logger')

signal.signal(signal.SIGINT, signal_handler)

# data files
folder = 'debug_robot'
if not os.path.exists(folder):
    os.makedirs(folder)

# date and time
header = time.strftime("%x").replace('/','-') + '_' + time.strftime("%X").replace(':','-')

file_sensed_force = open(folder + '/' + header + '_force.txt','w')

file_sensed_force.write('Sensed force\n')

# open redis server
r_server = redis.StrictRedis(host='localhost', port=6379, db=0)

# redis keys used in SAI2
# SIM_TIMESTAMP_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::timestamp";
EE_DESIRED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::desired_force";
# EE_FORCE_SENSOR_FORCE_KEY = "sai2::optoforceSensor::6Dsensor::force";
EE_SENSED_FORCE_LOGGED_KEY = "sai2::iiwaForceControl::iiwaBot::simulation::data_log::sensed_force";

# data logging frequency
logger_frequency = 1000.0  # Hz
logger_period = 1.0/logger_frequency
t_init = time.time()
t = t_init

print 'Start Logging Data\n'

while(runloop):
	t += logger_period

	force = json.loads(r_server.get(EE_SENSED_FORCE_LOGGED_KEY))
	desired_force = json.loads(r_server.get(EE_DESIRED_FORCE_LOGGED_KEY))

	# line = r_server.get(SIM_TIMESTAMP_KEY) + '\t' + " ".join([str(x) for x in force]) + '\t' + " ".join([str(x) for x in desired_force]) + '\n'
	line = '0\t' + " ".join([str(x) for x in force]) + '\t' + " ".join([str(x) for x in desired_force]) + '\n'
	# print line
	file_sensed_force.write(line)

	counter = counter + 1

	time.sleep(max(0.0,t-time.time()))

elapsed_time = time.time() - t_init
print "Elapsed time : ", elapsed_time, " seconds"
print "Loop cycles  : ", counter
print "Frequency    : ", counter/elapsed_time, " Hz"

# file_sensed_force.close()