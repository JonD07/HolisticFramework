import subprocess
import os, os.path, shutil
import time
import sys



# Path to the C++ executable
ORCHESTRATOR_PATH = "/home/jonathan/git/HolisticFramework/"
# Run parameters
NUM_PLOTS = 10
BATTERY_BUFFER = 0.05
TOTAL_BATTERY = 150000
BATTERY_SWAP = 120
DRONE_NUM = 1
INITIAL_ALPHA = 1.0
# INITIAL_V = 10.0
INITIAL_V = 7.5
# Mission planner algorithm
ALGORITHM = 3
ITERATIONS = 3
CONSISTENT_SPEED = True

mp_path = ORCHESTRATOR_PATH+"MissionPlanner/build/mission-planner"
sim_path = ORCHESTRATOR_PATH+"DroNS3/simulation.py"
exp_path = ORCHESTRATOR_PATH+"FW_Test/"
plan_path = ORCHESTRATOR_PATH+"plan/"
odom_path = ORCHESTRATOR_PATH+"odometry/"
sim_out_path = ORCHESTRATOR_PATH+"sim_out/"
fw_out_path = ORCHESTRATOR_PATH+"framework_out/"
python_path = "/home/jonathan/git/HolisticFramework/drone_env/bin/python"

'''
We are using drone 1, from "Looking before Crossing..." paper but with limited battery
3
# Max speed
10.0
# Usable Jules in battery (assuming 10.0Ah, 15.2v battery)
100000
# Battery swap time
120
# Energy profile ( c1x^{3} + c2x^{2} + c3x + c4 )
0.07 0.0391 -13.196 390.95
'''

def energy_used(v, t):
	power = (0.07)*v**3 + (0.0391)*v**2 + (-13.196)*v + (390.95)
	# Power is jule/seconds -> power*time = jules
	return power*t

# def total_battery():
# 	return TOTAL_BATTERY

def safe_battery():
	return TOTAL_BATTERY*(1-BATTERY_BUFFER)

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
	delete_files(sim_out_path)
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
	print("  Running Mission Planner:", mp_path, 'scenario_run.txt', str(alg), '1', '1', results_path, str(run_num))
	process = subprocess.Popen([mp_path, 'scenario_run.txt', str(alg), '1', '1', results_path, str(run_num)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	
	if stderr:
		print(f"\n*****\nError Running Global Planner: {stderr.decode()}\n*****\n")
	else:
		print("  Successfully Ran Mission Planner")
		with open("global_planner.out", 'a') as output_file:
			output_file.write("\n** New Run: **\n")
			output_file.write(stdout.decode("utf-8"))


# Function to run simulation and wait for it to finish
def run_simulation(sim_plan_path):
	# Count the number of sensors in this plan
	num_sensors = 0
	with open(sim_plan_path, 'r') as plan_file:
		for line in plan_file:
			line_list = line.split()
			if int(line_list[0]) == 5:
				num_sensors += 1
	# Time run
	start_time = time.time()
	# path-to-python, path-to-DroNS3-sim, path-to-plan
	process = subprocess.Popen([python_path, sim_path, sim_plan_path], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
	stdout, stderr = process.communicate()  # Waits for the executable to finish
	end_time = time.time()
	if stderr:
		# print(str(stdout))
		print(f"Error:\n{stderr.decode()}")
	else:
		print("  Successfully Ran Simulation")
	return end_time - start_time, num_sensors


def update_input_for_remaining_sensors(original_input_file, unvisited_ids, new_input_file):
	'''
	Reads the original scenario file and writes a new one containing ONLY 
	the base station and the unvisited sensors.
	'''
	# Convert all unvisited IDs to integers to safely handle both string and int inputs
	unvisited_ids_int = [int(sensor_id) for sensor_id in unvisited_ids]

	with open(original_input_file, 'r') as f:
		lines = f.readlines()

	# The first line contains the original number of sensors
	num_original_sensors = int(lines[0].strip())
	
	new_sensor_lines = []

	# Loop through the original sensors (Line 1 to num_original_sensors)
	# Sensor ID corresponds to loop index (0 to num_original_sensors - 1)
	for sensor_id in range(num_original_sensors):
		if sensor_id in unvisited_ids_int:
			new_sensor_lines.append(lines[sensor_id + 1])

	# The base station is located immediately after the sensors
	base_station_line = lines[num_original_sensors + 1]

	# Write the updated data to the new input file
	with open(new_input_file, 'w') as f:
		# Write the NEW number of sensors
		f.write(f"{len(new_sensor_lines)}\n")
		
		# Write only the unvisited sensors
		for line in new_sensor_lines:
			f.write(line)
			
		# Append the base station at the end
		f.write(base_station_line)


def run_baseline(input_file, fixed_alpha=1.0, fixed_v=INITIAL_V):
	current_input = input_file
	iteration = 0
	all_sensors_visited = False
	cumulative_time = 0
	cumulative_latency = 0
	total_sensors_collected = 0
	unvisited_file = sim_out_path+"unvisited_sensors.txt"

	while not all_sensors_visited:
		all_sensors_visited = True
		# Prepare scenario files
		prepare_standard_scenario(current_input, fixed_v, fixed_alpha)
		# Run the mission planner
		run_mission_planner(ALGORITHM, exp_path, 0)
		# How many plans did we generate? (there is a blank .temp file we need to ignore)
		num_plans = len([name for name in os.listdir(plan_path) if os.path.isfile(os.path.join(plan_path, name))]) - 1
		print(" Number of generated plans: ", num_plans)
		# For each generated plan...
		for i in range(num_plans):
			restart_planner = False
			# Run the simulator
			print(f" Running Simulator")
			r_time, sensors = run_simulation(plan_path+f"plan_0_{i}.pln")
			# Do we have left-over waypoints?
			if os.path.exists(unvisited_file):
				# Yes... Collect ALL unvisited waypoints
				restart_planner = True
				unvisited_ids = []
				# Get sensors from this mission
				with open(unvisited_file, 'r') as f:
					unvisited_ids = f.read().split()
				print(f"   Failed to visit: {unvisited_ids}")
				sensors -= len(unvisited_ids)
				# Get the sensors from plans (i+1) up to num_plans
				for j in range((i+1), num_plans):
					# Open the plan file in read mode
					with open(plan_path+f"plan_0_{j}.pln", "r") as file:
						# Loop through each line in the file
						for line in file:
							# Split the line into a list of strings based on whitespace
							parts = line.split()
							# Check if the line is not empty and if the first element is '5'
							if len(parts) > 0 and parts[0] == '5':
								# The sensor ID is the second element (index 1)
								sensor_id = parts[1]
								# Append the ID to our list (converting it to an integer is optional but recommended)
								unvisited_ids.append(int(sensor_id))
				# Clean up for next run
				os.remove(unvisited_file)
				# Update the input file
				current_input = exp_path + f"temp_input.txt"
				update_input_for_remaining_sensors(input_file, unvisited_ids, current_input)
				# Break out of for... retart while-loop
				all_sensors_visited = False

			# Record run stats
			total_sensors_collected += sensors
			if iteration > 1:
				cumulative_time += r_time + BATTERY_SWAP
			else:
				cumulative_time += r_time
			cumulative_latency += cumulative_time*sensors
			iteration += 1
			if restart_planner:
				break

	# If we hit this part... we hit every sensor!
	print("  All sensors successfully visited!")
	# Record final data for the baseline
	with open(fw_out_path+"baseline_stats.txt", "a") as f:
		f.write(f"{{sensors:{total_sensors_collected}, alpha:{fixed_alpha}, time:{cumulative_time}, average_lat:{cumulative_latency/total_sensors_collected}, sorties:{iteration}, input:{input_file}}}\n")


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
		print(f"   Plan Totals (J,v,t): ", total_energy, speed_total/speed_count, total_time)
		stat_list.append([total_energy, speed_total/speed_count, total_time])


# Determines if the plan is consistent. Returns T/F and a list with new v_j/alpha
def consistent(run_stats, v_j, alpha, i = 0):
	'''
	Analyzes energy used and average speed of the run_stats, which should be a list of lists, 
	where each nested list has total-energy, average-speed, and total-time of each time a 
	sub-tour was run. Returns consistency boolean and a list with the average speed and 
	recommended alpha. The consistency boolean will be True if no sub-tour went over the 
	allowed energy budget.
	'''
	print(" Checking Consistency", run_stats)
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
	print("  Total averages: ", avg_energy, avg_speed, avg_time)
	# Are we good on energy?
	if max_energy > safe_battery():
		good_plan = False
		# Update alpha (only when bad!) a_i+1 = a_i(desired-usage/actual-usage)
		parameters[1] = alpha*(1 - 0.75*(1 - safe_battery()/max_energy))
		print(f"   Max recorded energy {max_energy} higher than {safe_battery()}, update alpha to {parameters[1]}")
	# Error in speed (Not required for consistency)
	v_error = (v_j-avg_speed)/v_j
	if abs(v_error) > 0.25:
		print(f"  Average speed {avg_speed} not close to {v_j}!! error: {abs(v_error)}")
		if CONSISTENT_SPEED:
			# Update the stats
			good_plan = False
	elif abs(v_error) > 0.1:
		print(f"  Average speed {avg_speed} not close to {v_j}, error: {abs(v_error)}")
	parameters[0] = avg_speed

	# Record this data
	f = open(fw_out_path+"run_stats.txt", "a")
	f.write(f"Plan {i} speed: {avg_speed}, average-energy: {avg_energy}, time: {avg_time}\n")
	f.close()
	print("  * Consistent results:", good_plan, parameters)
	
	return good_plan, parameters


def run_framework(input_file, initial_alpha = INITIAL_ALPHA, initial_v = INITIAL_V, find_consistent = True):
	## Set initial v_j and alpha_j guess
	v_j = initial_v
	# v_j = 5.0
	alpha = initial_alpha
	# alpha = 0.7
	iteration = 0
	run_solver = True
	## While still not consistent
	while run_solver:
		run_solver = False
		iteration += 1
		# Record this data
		f = open(fw_out_path+"run_stats.txt", "a")
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
		# How many plans did we generate? (there is a blank .temp file we need to ignore)
		num_plans = len([name for name in os.listdir(plan_path) if os.path.isfile(os.path.join(plan_path, name))]) - 1
		print(" Number of generated plans: ", num_plans)
		total_time = 0
		cumulative_latency = 0
		total_sensors = 0
		## For each plan file generated:
		for i in range(num_plans):
			## For iterations
			stats = []
			r_time = 0
			sensors = 0
			for run in range(ITERATIONS):
				## Run simultor
				print(f" Running Simulator")
				r_time, sensors = run_simulation(plan_path+f"plan_0_{i}.pln")
				# Short sleep to let the sim fully finish
				time.sleep(2.5)
				## Collect average energy used, drone speed
				collect_run_stats(stats)
			## Are plans consistent?
			good_tour, new_stats = consistent(stats, v_j, alpha, i)
			# Record this data
			f = open(fw_out_path+"run_stats.txt", "a")
			f.write(f"Sensors: {sensors}, time: {r_time}\n")
			f.write(f"Consistent plan: {good_tour}, recommended parameters: {new_stats}\n")
			f.close()
			# Track the averages
			total_speeds += new_stats[0]
			total_alpha += new_stats[1]
			good_plan = good_plan and good_tour
			if i > 0:
				total_time+=r_time+BATTERY_SWAP
			else:
				total_time+=r_time
			cumulative_latency += total_time*sensors
			total_sensors += sensors
		# Do we run again..?
		if find_consistent:
			run_solver = not good_plan
		v_j = total_speeds/num_plans
		alpha = min(total_alpha/num_plans, alpha)
		# Record this data
		f = open(fw_out_path+"run_stats.txt", "a")
		f.write(f"Results: {good_plan} {total_sensors} {alpha} {v_j} {cumulative_latency/total_sensors} {total_time} {input_file} {iteration}\n")
		f.close()
		with open(fw_out_path+"fw_stats.txt", "a") as f:
			f.write(f"{{valid:{good_plan}, sensors:{total_sensors}, alpha:{alpha}, time:{total_time}, average_lat:{cumulative_latency/total_sensors}, sorties:{num_plans}, iterations:{iteration}, input:{input_file}}}\n")
	return alpha, v_j

		


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

	if len(sys.argv) == 2:
		if sys.argv[1] == "framework":
			print("(Standard) Running all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					print(f"Running framework on {input_file}")
					# Record this data
					f = open(fw_out_path+"run_stats.txt", "a")
					f.write(f"Running framework on {input_file}\n")
					f.close()
					# Run our algorithm
					run_framework(input_file)
			print("(Alpha fixed at 0.75) Running all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					print(f"Running framework on {input_file}")
					# Record this data
					f = open(fw_out_path+"run_stats.txt", "a")
					f.write(f"Running framework on {input_file}\n")
					f.close()
					# Run our algorithm
					run_framework(input_file,initial_alpha=0.75,find_consistent=False)
			print("(Alpha fixed at 0.5) Running all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					print(f"Running framework on {input_file}")
					# Record this data
					f = open(fw_out_path+"run_stats.txt", "a")
					f.write(f"Running framework on {input_file}\n")
					f.close()
					# Run our algorithm
					run_framework(input_file,initial_alpha=0.5,find_consistent=False)

		if sys.argv[1] == "performance-reactive":
			print("Reactive baseline (Alpha = 1) with all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					for run in range(ITERATIONS):
						print(f" => Starting Baseline Run for {input_file}, run {run}")
						run_baseline(input_file, fixed_alpha=1.0)
			print("Reactive baseline (Alpha = 0.75) with all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					for run in range(ITERATIONS):
						print(f" => Starting Baseline Run for {input_file}, run {run}")
						run_baseline(input_file, fixed_alpha=0.75)
			print("Reactive baseline (Alpha = 0.5) with all inputs in:", exp_path)
			for n in range(5, 31, 5):
				for i in range(NUM_PLOTS):
					input_file = exp_path+f"plot_{n}_{i}.txt"
					for run in range(ITERATIONS):
						print(f" => Starting Baseline Run for {input_file}, run {run}")
						run_baseline(input_file, fixed_alpha=0.5)

		if sys.argv[1] == "performance-framework":
			print("(Standard) Running all inputs in:", exp_path)
			for n in range(25, 31, 5):
				for i in range(NUM_PLOTS):
					for run in range(ITERATIONS):
						input_file = exp_path+f"plot_{n}_{i}.txt"
						print(f"Determining parameters for {input_file}")
						# Record this data
						f = open(fw_out_path+"run_stats.txt", "a")
						f.write(f"Running framework on {input_file}\n")
						f.close()
						# Run the framework
						alpha, v = run_framework(input_file)
						# Using the found alpha, v, run the planner/sim again..
						print(f"Found {alpha}:{v}, Running {input_file}\n")
						run_baseline(input_file=input_file, fixed_alpha=alpha, fixed_v = v)

		else:
			print("Running framework on single input:", sys.argv[1])
			# Record this data
			f = open(fw_out_path+"run_stats.txt", "a")
			f.write(f"Running framework on {sys.argv[1]}\n")
			f.close()
			# Run our algorithm
			run_framework(sys.argv[1])
	else:
		print("Unexpected arguments...")
