# Holistic Planning Framework

This is the Holistic Planning Framework described in the paper "*Holistic Path Planning for UAV-Assisted Data Collection for IoT Systems*."

## Using Framework

### Prerequisites
You must first download clone the following git repositories to use this framework:

https://github.com/JonD07/MissionPlanner

https://github.com/pervasive-computing-systems-group/DroNS3

We recommend following the setup cloning these projects into the root directory of this repository. E.g.:

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


Please follow the setup instructions for each of these repositories.

### Running Framework
Update `ORCHESTRATOR_PATH` in the `Run_Framework.py` script so that it points to the location of this repository in your file system. If you chose to clone the above repositories to a different location and in the root directory of this repository, then you will also need to update `mp_path`, `sim_path`, and `exp_path`.

You can also update run parameters at the top of the `Run_Framework.py` script.
