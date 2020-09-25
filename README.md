# CARLA Bike Simulator Tools

This project contains code and Unreal Engine assets for using CARLA as a bicycle simulator.

The folowing folder structure is used:
* ./
    * {CARLA_VERSION}
        * Scripts
        * UE Assets
    * ...

In order to run the scripts, they need to be able to locate the carla python module.
This can be ensured in one of two ways:
* Copy the scripts to the folder ```{CARLA_ROOT}/PythonAPI/examples```, or
* Install the carla package (corresponding to the correct CARLA version) in the python
environment being used

# Project Info
Initial code written as part of the Master's Thesis of Paul Pabst at the
Chair of Traffic Engineering and Control at the Technical University of Munich (TUM).

Code maintained by the Chair of Traffic Engineering and Control at TUM.