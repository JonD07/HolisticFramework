import subprocess
import os, os.path, shutil
import time


# Path to the C++ executable
ORCHESTRATOR_PATH = "/home/jonathan/Research/HolisticFramework/Orchestrator/"
# Run parameters
NUM_PLOTS = 10
BATTERY_BUFFER = 0.05
DRONE_NUM = 1
INITIAL_ALPHA = 1.0
INITIAL_V = 10.0
# Mission planner algorithm
ALGORITHM = 3
ITERATIONS = 3

mp_path = ORCHESTRATOR_PATH+"MissionPlanner/build/mission-planner"
sim_path = ORCHESTRATOR_PATH+"DroNS3/simulation.py"
exp_path = ORCHESTRATOR_PATH+"MissionPlanner/test/FW_Test/"
plan_path = ORCHESTRATOR_PATH+"plan/"
odom_path = ORCHESTRATOR_PATH+"odometry/"
python_path = "/usr/bin/python3.8"

'''
We are using drone 1, from "Looking before Crossing..." paper but with limited battery
3
# Max speed
10.0
# Usable Jules in battery (assuming 10.0Ah, 15.2v battery)
75000
# Battery swap time
120
# Energy profile ( c1x^{3} + c2x^{2} + c3x + c4 )
0.07 0.0391 -13.196 390.95
'''

def energy_used(v, t):
	power = (0.07)*v**3 + (0.0391)*v**2 + (-13.196)*v + (390.95)
	# Power is jule/seconds -> power*time = jules
	return power*t

def total_battery():
	return 75000

def safe_battery():
	return total_battery()*(1-BATTERY_BUFFER)

def delete_files(folder_path):
	'''
	Deletes all files in the given directory. This is used to remove hang-over files that might 
	be messing with things should some part of the framework fail. This will skip the .temp file, 
	which is used at times as a dummy place-holder for git.
	'''
	# Clear the old plan files
	for filename in os.listdir(folder_path):
		if filename != ".temp":
			file_path = os.path.join(folder_path, filename)
			try:
				if os.path.isfile(file_path) or os.path.islink(file_path):
					os.unlink(file_path)
				elif os.path.isdir(file_path):
					shutil.rmtree(file_path)
			except Exception as e:
				print('Failed to delete %s. Reason: %s' % (file_path, e))

# Function to read scenario.txt and write to scenario_run.txt
def prepare_standard_scenario(input_file_location, v, alpha):
	# Clear the old plan files
	delete_files(plan_path)
	# Read from scenario.txt
	with open('scenario.txt', 'r') as scenario_file:
		content = scenario_file.read()
	# Add the location of the input
	content += input_file_location+"\n"
	# Add in drone
	content += f"{DRONE_NUM}\n"
	# Write the online setup file (used for online planner/DroNS3)
	with open('scenario_online_setup.txt', 'w') as scenario_run_file:
		scenario_run_file.write(content)
	# Add in drone details
	content += f"{v} {alpha}"
	# Write to scenario_run.txt
	with open('scenario_run.txt', 'w') as scenario_run_file:
		scenario_run_file.write(content)


# Function to run mission planner and wait for it to finish
def run_mission_planner(alg, results_path, run_num):
	# scenario-file alg plan-flag results-flag results-path run-num
	process = subprocess.Popen([mp_path, 'scenario_run.txt', str(alg), '1', '1', results_path, str(run_num)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	
	if stderr:
		print(f"Error:\n{stderr.decode()}")
	else:
		print("Successfully Ran Mission Planner")
		print(stdout)


# Function to run simulation and wait for it to finish
def run_simulation(sim_plan_path):
	# path-to-python, path-to-DroNS3-sim, path-to-plan
	process = subprocess.Popen([python_path, sim_path, sim_plan_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	if stderr:
		print(f"Error:\n{stderr.decode()}")
	else:
		print("Successfully Ran Simulation")
		print(stdout)


def collect_run_stats(stat_list):
	'''
	Pushes a new list of run stats onto the stat_list. The run stats list has total-energy, 
	average-speed, and total-time (in that order)
	'''
	total_energy = 0
	speed_total = 0
	speed_count = 0
	total_time = 0
	with open(odom_path+'odometer_measurements.txt', 'r') as scenario_file:
		file_content = scenario_file.read()
		# print(file_content)
		odom_data = list(file_content.split('\n'))
		# Ignore first two line (arming and take-off)
		line_i = 0
		while line_i < len(odom_data):
			if len(odom_data[line_i]) > 2:
				dt = float(odom_data[line_i].split()[0])
				dx = float(odom_data[line_i].split()[1])
				flag = int(odom_data[line_i].split()[2])
				speed = dx/dt
				energy = energy_used(speed, dt)
				if flag == 0:
					# Hovering command
					total_energy += energy
					total_time += dt
				elif flag == 1:
					# Move command
					total_energy += energy
					speed_total += speed
					speed_count += 1
					total_time += dt
			line_i += 1
		print(f"Plan {i} Totals (J,v,t): ", total_energy, speed_total/speed_count, total_time)
		stat_list.append([total_energy, speed_total/speed_count, total_time])


# Determines if the plan is consistent. Returns T/F and a list with new v_j/alpha
def consistent(run_stats, v_j, alpha):
	'''
	Analyzes energy used and average speed of the run_stats, which should be a list of lists, 
	where each nested list has total-energy, average-speed, and total-time of each time a 
	sub-tour was run. Returns consistency boolean and a list with the average speed and 
	recommended alpha. The consistency boolean will be True if no sub-tour went over the 
	allowed energy budget.
	'''
	print("Checking Consistency", run_stats)
	# Assume good until otherwise stated..
	good_plan = True
	parameters = [v_j, alpha]
	# Determine the average accross all runs
	energy_total = 0
	speed_total = 0
	time_total = 0
	max_energy = 0
	for stat in run_stats:
		energy_total += stat[0]
		speed_total += stat[1]
		time_total += stat[2]
		if stat[0] > max_energy:
			max_energy = stat[0]
	avg_energy = energy_total/len(run_stats)
	avg_speed = speed_total/len(run_stats)
	avg_time = time_total/len(run_stats)
	print("Total averages: ", avg_energy, avg_speed, avg_time)
	# Are we good on energy?
	if max_energy > safe_battery():
		good_plan = False
		# Update alpha (only when bad!) a_i+1 = a_i(desired-usage/actual-usage)
		parameters[1] = alpha*(1 - 0.5*(1 - safe_battery()/avg_energy))
		print(f"Average energy {avg_energy} higher than {safe_battery()}, update alpha to {parameters[1]}")
	# Error in speed (Not required for consistency)
	v_error = (v_j-avg_speed)/v_j
	if abs(v_error) > 0.1:
		print(f"Average speed {avg_speed} not close to {v_j}, error: {abs(v_error)}")
		# Update the stats
	parameters[0] = avg_speed
	# Record this data
	f = open("run_stats.txt", "a")
	f.write(f"{avg_energy} {avg_speed} {avg_time}\n")
	f.close()
	print("Consistent results:", good_plan, parameters)

	return good_plan, parameters


def run_framework(input_file):
	## Set initial v_j and alpha_j guess
	v_j = INITIAL_V
	alpha = INITIAL_ALPHA
	iteration = 0
	run_solver = True
	## While still not consistent
	while run_solver:
		run_solver = False
		iteration += 1
		# Record this data
		f = open("run_stats.txt", "a")
		f.write(f"{alpha} {v_j}\n")
		f.close()
		## Run solver
		# Generate run file
		prepare_standard_scenario(input_file, v_j, alpha)
		# Run the solver
		print(f" Running Mission Planner, iteration: {iteration}")
		run_mission_planner(ALGORITHM, exp_path, 0)
		good_plan = True
		total_speeds = 0
		total_alpha = 0
		## For each plan file generated:
		num_plans = len([name for name in os.listdir(plan_path) if os.path.isfile(os.path.join(plan_path, name))])
		print("Number of generated plans: ", num_plans)
		for i in range(num_plans):
			## For iterations
			stats = []
			for run in range(ITERATIONS):
				## Run simultor
				print(f" Running Simulator")
				run_simulation(plan_path+f"plan_0_{i}.pln")
				# Short sleep to let the sim fully finish
				time.sleep(2.5)
				## Collect average energy used, drone speed
				collect_run_stats(stats)
			## Are plans consistent?
			good_tour, new_stats = consistent(stats, v_j, alpha)
			# Track the averages
			total_speeds += new_stats[0]
			total_alpha += new_stats[1]
			good_plan = good_plan and good_tour
		# Do we run again..?
		run_solver = not good_plan
		v_j = total_speeds/num_plans
		alpha = min(total_alpha/num_plans, alpha)


if __name__ == '__main__':
	## The basic algorithm:
	# Set initial v_j and alpha_j guess
	# Run solver
	# For each plan file generated:
	#   For 5? iterations
	#     Run simulator
	#     Collect average energy used, drone speed
	#   ENDFOR
	# ENDFOR
	# Are plans consistent?
	# No -> GoTO step 2
	# Yes -> Finished!
	'''
	Set initial v_j, alpha_j
	while not consistent
	  Run Global Planner
	  for iterations
	    run simulator
	  End-for
	  update v_j, alpha_j
	End-while
	'''

	for n in range(5, 31, 5):
		for i in range(NUM_PLOTS):
			input_file = exp_path+f"plot_{n}_{i}.txt"
			print(f"Running framework on {input_file}")
			# Record this data
			f = open("run_stats.txt", "a")
			f.write(f"Running framework on {input_file}\n")
			f.close()
			# Run our algorithm
			run_framework(input_file)
