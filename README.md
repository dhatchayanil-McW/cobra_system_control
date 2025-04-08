# M30 Lidar System Control

This repo provides the classes and functions necessary to communicate with, configure, and control Lumotive's lidar systems.

## Python setup
This repo uses Python3.10, the version supported the Lumotive Compute Platform (NCB). The public packages available for use without modifying the build are available in the `requirements_nxp.txt` file.


## Repository Structure
Here is a description of each of the top-level directories, plus some highlighted content:

1. `bringup_scripts/`: Contains scripts that at one point in time, were useful for system and PCB bringup. These files are not guaranteed to work on the most recent HW and could potentially damage a system. Do not run before inspecting the contents and updating.
1. `cobra_system_control/`: contains the Python3.10 modules that are available when this repo is pip-installed.
   1. `bash_tools/`: contains all Bash (.sh) scripts
      1. `deploy.sh`: Deploys the Cobra remote Pyro5 object using `host_cobra.py`
      1. `sync_fpga_build.sh`: Syncs FPGA design collateral to this repo.
   1. `boot_scripts/`
      1. `host_cobra.py`: Performs system discovery and opens correct configuration of Cobra object. After successful creation of the object, it is launched as a Pyro5 remote object.
      1. files related to setting up systemd services live here.
   1. `resources/`: Contains MCS files related to the FPGA bitstream and LCM voltage patterns, as well as YAML memory map data files.
   1. `scripts/`: Scripts to help perform various operations helpful during development and testing.
      1. `start_lidar.py`: Start the lidar depth measurements with configurable settings.
1. `unittests/`: contains Python pytest modules for unit and integration testing of hardware.


## Cobra remote object
The Cobra remote object is actively hosted using Pyro5 for remote access and facilitates interaction with the M30 lidar hardware over the network. This is usually accomplished through the `remote` service, or the `deploy.sh` shell script at the top level of `cobra_system_control`. Both of these delegate to the `host_cobra.py` script in `boot_scripts/`.

The `cobra_lidar_api` interacts directly with the Cobra remote object to call methods on the Cobra and SensorHead classes.


### Using in Development
**Caution**, actively developing Cobra and its subsystems while using the remote object can be tricky. For any Cobra, SensorHead, or Device code changes to be reflected in the Cobra remote object, the object must be restarted.

If the object is hosted via systemd, you need to execute:
`systemctl restart remote`

If the object is hosted via `deploy.sh`, then `ctrl+c` and run the script again.


### Acquiring Cobra remote on the compute platform
To acquire the remote Cobra object, use the `Cobra.remote` class method, with no arguments. This call will attempt to locate the object on the local machine.

```
with Cobra.remote() as c:
    print(f'{c.sen.debug.read_fields("git_sha"):#010x}')
```

### Acquiring Cobra remote on your PC
Using your personal machine requires a SSH tunnel with port forwarding to "pretend" like your machine is communicating the the remote object on the Compute Platform. This is set up on production releases using username and password arguments in the remote() call.


## Syncing FPGA build artifacts
The FPGA build artifacts are in a folder with the format `<project name>_fpga-<git sha>-<version tag>-<commits after tag>-g<git_sha>`.

The MCS file and yamls related to the build can be synced over to cobra-system-control by running: `. sync_fpga_build.sh <path to FPGA build directory>`

MCS files are also converted to binary using the command: `hex2bin.py --pad FF --range 0: <file>.mcs <file>.bin`

Note, the `impl_1` and `yaml` folders must be at the first level under `<path to FPGA build directory>` when running this script.

In the future, the build artifacts will be moving to Artifactory and this process may need to change.


## Syncing LCM voltage pattern artifacts
Typically, updated LCM voltage patterns are delivered in MCS format. The MCS can be synced over to cobra-system-control by running `. sync_lcm_patterns.sh <path to LCM pattern MCS>` from the `bash_tools` folder. This script will move the MCS to the `resources/` directory and create an BIN version.
