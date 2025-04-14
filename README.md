# Holistic Planning Framework

This is the Holistic Planning Framework described in the paper "*Holistic Path Planning for UAV-Assisted Data Collection for IoT Systems*."

## Using Framework

### Prerequisites
You must first download/clone the following git repositories to use this framework:

https://github.com/JonD07/MissionPlanner

https://github.com/pervasive-computing-systems-group/DroNS3

You must checkout the dev/HolisticPlanner branch of the DroNS3 repository. We recommend cloning these projects into the root directory of this repository. E.g.:

```
HolisticFramework
│   README.md
│   ...
│
└───DroNS3
│   │   ...
│   
└───MissionPlanner
│   │   ...
```


Please follow the setup instructions for each of these repositories. Note that you must build the Mission Planner project and the SimpleNetSim in DroNS3.

### Setting Parameters
Update the file paths in the following locations so that they match your own file structure:

- `scenario.txt`
- `scenario_online_setup.txt`
- `DroNS3/defines.py`
- `ORCHESTRATOR_PATH` and `python_path` in the `Run_Framework.py`

If you chose to clone the above repositories to a different location than the root directory of this repository, then you will also need to update `mp_path`, `sim_path`, and `exp_path` in the `Run_Framework.py` script.

You can also update run parameters at the top of the `Run_Framework.py` script, such as the number of input files to run, the battery safety buffer, or which mission planner algorithm to use.

### Running Framework

You need to launch the ardupilot simulator before running the framework. Let the simulator startup and get "settled" before running the framework.

Assuming that you are launching the drone from the CSM survey field (and have added `CSM_SurveyField=39.739550,-105.222467,1812,0` to the `ardupilot/Tools/autotest/locations.txt` file), then you can launch the drone simulator using the following (from the ardupilot root directory):

```
cd ArduCopter
. ~/.profile
sim_vehicle.py -f quad -L CSM_SurveyField --console --map --osd
```

Once the simulator has fully launched, launch the framework in a new tab. To run a single problem input (for example, the field prototype from the paper), pass the `Run_Framework.py` script the location of the input file:

```
python Run_Framework.py Field_Inputs/field_exp.txt
```

To run the full framework experiment, run the script without any arguments:

```
python Run_Framework.py
```
