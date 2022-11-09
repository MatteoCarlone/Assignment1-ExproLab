

# Surveillance Robot Architecture

**A ROS-based Architecture of a surveillance robot based on a ROS-SMACH Finite State Machine and the use aRMOR to manage Ontology desciption in ROS.**  

Author: *Matteo Carlone s4652067*

---

## Scenario
The scenario involves a surveillance robot that moves in a 2D environment comprehensive of locations (simple rooms and corridors) connected trough doors. 
More in detail: 
	- corridors are simply rooms with more than one door as access point.
	- if a room is not visited for a fixed amount of time will become urgent.

The robot behave as follows:

- The robot start its motion in a Starting Location where a doc for its battery recharge is located.

- The robot moves with the following surveillance policy:
	
	- it mainly prefer moving in corridors

	- if the robot can reach of or more urgent locations it will go for one of them

	- if the robot's battery is low and it can reach the DOC-Station (Staring Room) it will go for it 

	- if the robot's battery is low but it can not reach the DOC-Station (Staring Room) it will move as the battery is full

*Image of the Environment*

...........

`caption` :
	
	- Sample environment with cartesian coordinates associated with each room.

### Assumptions

For simplicity and showing purposes, we consider a scenario with the following assumptions.

* The robot moves in a 2D environment without obstacles.

* Given a current and target position, the robot plans a trajectory to follow, i.e., a list of n via points (default n = 10) equally spacied.
  Then it follows the trajectory by reaching each via point.

* The battery can become low at any time, and the robot should immediately react to this event in two ways:

	* if the robot's battery is low and it is in a room connected to the DOC-Station (Starting Room), the robot will go for it and recharge.

	* if the robot's battery is low but the room is not connected to the DOC-Station (Starting Room), the  robot will move as the battery is full.

* Every room become urgent if it is not visited for a predefined time interval (7 sec).

* To avoid the possibility of having urgent rooms after a several amount of time I assumed the every room being visited at the same time at zero time instant.

* Starting Location of the robot is assumed to be Corridor.

## Project Structure

### Package list 

This repository contains a ROS package named `exprolab_1` that includes the following resources.
 - [CMakeList.txt](CMakeList.txt): File to configure this package.
 - [package.xml](package.xml): File to configure this package.
 - [setup.py](setup.py): File to `import` python modules from the `utilities` folder into the 
   files in the `script` folder. 
 - [launcher/](launcher/): Contains the configuration to launch this package.
    - [run.launch](launch/run.launch): It launches this package by spawning x-term terminals to
     visualize every ros-node outputs
 - [msg/](msg/): It contains the message exchanged through ROS topics.
    - [Point.msg](msg/Point.msg): It is the message representing a 2D point.
 - [srv/](srv/): It Contains the definition of each server used by this software.
    - [Reason.srv](srv/Reason.srv): It defines the request and response to start up the reasoning process, it response with room to be pointed 
 - [action/](action/): It contains the definition of each action server used by this software.
    - [Plan.action](action/Plan.action): It defines the goal, feedback and results concerning 
      motion planning.
    - [Control.action](action/Control.action): It defines the goal, feedback and results 
      concerning motion controlling.
 - [scripts/](scripts/): It contains the implementation of each software components.
    - [fsm.py](scripts/fsm.py): 
    - [initial_state.py](scripts/initial_state.py): 
    - [reasoner.py](scripts/reasoner.py): 
    - [battery.py](scripts/battery.py): 
    - [planner.py](scripts/planner.py): It is a dummy implementation of a motion planner.
    - [controller.py](scripts/controller.py): It is a dummy implementation of a motion 
      controller.

 - [utilities/exprolab_1/](utilities/exprolab_1/): It contains auxiliary python files, 
   which are exploited by the files in the `scripts` folder.
    - [environment.py](utilities/exprolab_1/environment.py): It contains the name 
      of each *node*, *topic*, *server*, *actions* and *parameters* used in this architecture.
    - [helper.py](utilities/exprolab_1/helper.py): It contains a so called InterfaceHelper Class that implements all the ros-srv and ros-action clients, 
      and other auxiliary functions to manage the fsm transition and other global procedures.
    - [ActionHelper.py](utilities/exprolab_1/ActionHelper.py): It contains a so called ActionHelper client usefull to simply manage ROS-Actions
    
 - [diagrams/](diagrams/): It contains the diagrams shown below in this README file.

### Dependencies

## Software Components

## Launching the Sowtware
