# Surveillance Robot Architecture

**A ROS-based Architecture of a surveillance robot based on a ROS-SMACH Finite State Machine and the use aRMOR to manage Ontology description in ROS.**  

---

## Introduction

This project is the first assignment of the Robotics Engineering Course's Experimental Robotics Laboratory at the University of Genoa.
The task entails developing a software architecture for surveillance robots. 
The project aims to deepen the ROS (Robot-Operating-System) utilization by leveraging:

- [***SMACH***](http://wiki.ros.org/smach), a powerful and scalable Python-based library for hierarchical state machines.

- [***aRMOR***](https://github.com/EmaroLab/armor), a robust and adaptable management solution for single and multi-ontology architectures running on ROS. It supports the loading, querying, and modification of several ontologies.


## Project Scenario

The scenario involves a surveillance robot that moves in a 2D environment comprehensive of locations (simple rooms and corridors) connected trough doors. 
More in detail: 
- corridors are simply rooms with more than one door as access point.
- if a room is not visited for a fixed amount of time will become urgent.

**The robot generally behave as follow:**

- The robot start its motion in a Starting Location where a doc for its battery recharge is located.

- The robot moves with the following ***surveillance policy***:
	
	- it mainly prefer moving in corridors

	- if the robot can reach of or more urgent locations it will go for one of them

	- if the robot's battery is low and it can reach the DOC-Station (Staring Room) it will go for it 

	- if the robot's battery is low but it can not reach the DOC-Station (Staring Room) it will move as the battery is full

*Image of the Environment*

<p align="center">
<img
	src="/img/Env.png"
	title="Environment img"
	width="400">
</p>

`caption` :

		- Sample environment with cartesian coordinates associated with each room.


## Project Structure 

The following list includes a brief overview of each project component as well as their arrangement into directories and subfolders.

<details>
  <summary> Package list </summary>

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
    - [fsm.py](scripts/fsm.py): The Script implementing the SMACH Finite State Machine.
    - [initial_state.py](scripts/initial_state.py): The Script implementing the initial load and initialization of the Ontology.
    - [reasoner.py](scripts/reasoner.py): The Script implementing the reasoning the room to be guarded.
    - [battery.py](scripts/battery.py): The Script implenting the battery and charging behaviour of the robot.
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
 - [img/](img/): It contains the images shown in this README file

 </details>

### Software Components

It follows the details of each software component implemented in this repository, which is available in the [scripts folder](scripts/).

#### The `F.S.M` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/FSM.png"
	title="FSM img"
	width="600"> 
</p>

This Node implement a SMACH Finite-State-Machine that guide all the surveillance routine. 
The communication with other nodes is managed using ros-services and ros-actions, the actual structure of the service and action client is implemented in the script [helper.py](utilities/helper.py) in the [utilities folder](utilities/). 
two sub state machines are implemented to respectively exploit the motion and the robot's batttery recharge:

* Moving Sub-State-Machine: 
It represent the motion routine composed by a pianification of th path to follow from one room to another and a robot controllier state in which the robot actualy achieve the target room. 

* Recharge Sub-State-Machine: 
It represent the recharge routine composed by a movement to the DOC-Station (Starting Room) which is actualy another instance of the aformentioned Moving Sub-State-Machine (MOVE_TO_DOCK) and the recharging of the robot's battery.

*SMACH FSM Representation*

<p align="center">
<img
	src="/img/Smach_FSM.png"
	title="FSM_Smach img"
	width="600">  
</p>


`caption` :

	- Auto generated representation of the SMACH state machine comprehensive of the sub-state-machines.

------

#### The `Initial State` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/Initial_State.png"
	title="Initial State img"
	width="500">
</p>
 
This node, representing the initial state, is called by the FSM via the Start service request. Once called it basically load a predefined empty ontology from the [topology folder](topology/), than it uses an ARMOR action client to manipulate the ontology and define all the enviroment individuals and features:
* The Starting Room of the robot.
* The current time associated with the robot.
* all the connection between rooms and so:
	* the doors of every room.
	* weather a room is a corridor or not.
* As defined in the assumptions, the visited time of every room is here set to the current time.

An Empty response is returned to notify the FSM that the ontology initialization is finished.
the [helper.py](utilities/helper.py) script in the [utilities folder](utilities/) actualy implement the Start service client and transform the empy start response into a transition understandable by the FSM.

------

#### The `Reasoner` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/Reasoner.png"
	title="Reasoner img"
	width="500">
</p>

This node, representing the reasoner state, is called by the FSM via a the Reason custom service request. Once called it updates some informations directly form the ontology:

* The Robot current location
* The current time instant associated with the Robot
* The rooms that the Robot can reach
* The auto-generated list of Urgent rooms 
* The list of Corridors set in the environment

Then it basically computes the next room to point by following the aformentioned surveillance policy.
It returns the target room in the Reason service response. The [helper.py](utilities/helper.py) script in the [utilities folder](utilities/), that implement the Reason client, get the response and notifies the FSM that the reasoning state has finished.

------

#### The `Planner` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/Planner.png"
	title="Planner img"
	width="600">
</p>

This node, representing the planning state, is called by the Moving sub-state machine via a Control action-client request. Once called it retrive the robot current location and has from the request the target room. It basically just generate a set of n (default n = 10) equally spacied via points 
from the current robot's location coordinates to the target ones. This operation loses time to simulate the operation.
the via points are returned as Plan action response.
The [helper.py](utilities/helper.py) script in the [utilities folder](utilities/), that implement the Plan action-client, get the Plan action response, stores the via points and notifies the FSM that the planning state has finished.

------

#### The `Controller` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/Controller.png"
	title="Controller img"
	width="600">
</p>

This node, representing the controlling state, is called by the Moving sub-state machine via a Plan action-client request. Once called it retrive the via points generated in the planning state. It basically lose time and print the via points on screen to simulate the robot motion in the environment. When the robot get to the target position it upadates the ontology by replacing:
* the current robot position with the target room
* the current time instant associated with the robot
* the time in which the target room has been visited
the reached room is returned as Control action response.
The [helper.py](utilities/helper.py) script in the [utilities folder](utilities/), that implement the Control action-client, get the Control action response, and notifies the FSM that the Controlling state has finished.

------

#### The `Battery` Node, its Message and Parameters.

<p align="center">
<img
	src="/img/Battery.png"
	title="Battery img"
	width="500">
</p>

This node, representing the recharge state, is called by the Recharge sub-state machine via a Recharge service request. 
Before being called, when this node start a random notifier for the state of the battery takes action in another thread.
The random notifier basically send an asynchronous Boolean message on the topic /battery_low representing that the robot's battery is low.
Once this node is called it start a loading bar animation that lose time and simulate the recharging procedure.
It returns an empty recharge service response.
The [helper.py](utilities/helper.py) script in the [utilities folder](utilities/), that implement the recharge service client, notifies the FSM that the recharging state has finished.

------

### Diagrams

The whole architecture is well described in the following UML Diagrams.


<p align="center">
<img
	src="/img/UML.png"
	title="UML img"
	width="600" height="650">
</p>

`caption` :

	- The whole UML Component diagram to have a more general and connected view of the previosuly presented softaìware components.

<p align="center">
<img
	src="/img/Time_Sequence.png"
	title="Time_Sequence img"
	width="600"> 
</p>

`caption` :

	- Time Squence Diagram of the project, with a draft of communication transitions between states.

### Dependencies

The software exploits [roslaunch](http://wiki.ros.org/roslaunch) and 
[rospy](http://wiki.ros.org/rospy) for using python with ROS. Rospy allows defining ROS nodes, 
services and related messages.

Also, the software uses [actionlib](http://wiki.ros.org/actionlib/DetailedDescription) to define action servers. In particular, this software is based on 
[SimpleActionServer](http://docs.ros.org/en/jade/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a2013e3b4a6a3cb0b77bb31403e26f137) and [SimpleActionClient](https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html) especially for the Controller and Planner behaviour.

The Finite States Machine and the two relative sub-state-machines are based on [SMACH](http://wiki.ros.org/smach).  the [smach_viewer](http://wiki.ros.org/smach_viewer) has been used for the fsm visualization in the software component section.
it is basically a node to visualize and debug the implemented Finite States Machines.

I used [aRMOR api](https://github.com/EmaroLab/armor_py_api) to manage the ontology description of this project. 
[aRMOR](https://github.com/EmaroLab/armor) is a package for managing sigle and also multi ontology aschitectures under ROS

## Launching the Software

This software has been based on ROS Noetic, and it has been developed with this Docker-based
[environment](https://hub.docker.com/repository/docker/carms84/exproblab), which already 
provides the required dependencies listed above. 

### Installation

Follow these steps to install the software.
 - Clone this repository inside your ROS workspace (which should be sourced in your `.bashrc`).
 - Run `chmod +x <file_name>` for each file inside the `scripts` folder.
 - Run `catkin_make` from the root of your ROS workspace.
 - Install `xterm` by entering the command `sudo apt install -y xterm`.
 - Install `smach` by entering the command `sudo apt-get install ros-noetic-smach-ros`
 - Install `aRMOR` directly from [here](https://github.com/EmaroLab/ros_multi_ontology_references.git). or manually download the latest release of each module from the following repositories:
	- [***ARMOR***](https://github.com/EmaroLab/armor)
	+ [***AMOR***](https://github.com/EmaroLab/multi_ontology_reference)
	+ [***armor_msgs***](https://github.com/EmaroLab/armor_msgs)

 - Clone the [`armor_api`](https://github.com/buoncubi/armor_py_api) repository in your workspace

 Note: [Here](http://emarolab.github.io/armor_py_api/) you can find the full documentation of the [`armor_api`](https://github.com/buoncubi/armor_py_api).

### Launchers

Use the following command to launch the software a spawn some xterm windows related to all the aformentioned software components.

```bash
roslaunch exprolab_1 run.launch
```

Note that the program runs in automatically in a loop and there's no need of the user to start up the finite state machine's transitions.

Check the `roslaunch` outcome to get the path where logs are stored. usually, it is `~/.ros/log/`.
That folder should also contain a link to the `latest` produced log.

If needed is of course possbile of running each node separetely but remember to launch the armor server node:
```bash
rosrun armor execute it.emarolab.armor.ARMORMainService
```

### Video of the project behaviour

<p align="center">
<img src="/img/behaviour.gif" width="600">
</p>

## Working Hypothesis

### Assumptions and System Features

For simplicity and showing purposes, we consider a scenario with the following assumptions.

* The robot moves in a 2D environment without obstacles.

* Given a current and target position, the robot plans a trajectory to follow, i.e., a list of n via points (default n = 10) equally spacied.
Then it follows the trajectory by reaching each via point.

* Every room that has multiple doors is assumed to be a corridor.

* Every room become urgent if it is not visited for a predefined time interval (7 sec).

* To avoid the possibility of having urgent rooms after a several amount of time I assumed the every room being visited at the same time at zero time instant.

* Starting Location of the robot is assumed to be a room and so a Corridor.

* The battery can become low at any time, and the robot should immediately react to this event in two ways:

	* if the robot's battery is low and it is in a room connected to the DOC-Station (Starting Room), the robot will go for it and recharge.

	* if the robot's battery is low but the room is not connected to the DOC-Station (Starting Room), the  robot will move as the battery is full.

* The Environment (in [Figure](img/Env.png)) is just a sample representation, in the [environment.py](utilities/environment.py) script ([utilities folder](utilities/)) is possibile to change rooms, coordinates and the staring room, while in the [initial_state.py](scripts/initial_state.py) script ([scripts folder](scripts/)) is possbile to change the connections between rooms.

* When the battery is low , as mentioned in the surveillance policy, the robot is forced to the DOC-Station (Starting Room) if and only if it can reach it, so I assume, especially in other environments, that there's a random possbility of continuing moving around without reaching the DOC. 

### System's limitations

The main limitation of this work is related to the last assumption since having the robot moving randomly with low battery, in a real word application could lead to a fully discharged. Get in this situation with the default environment is difficult beacuse of the way it was designed, but this still remains a hard limitation. Other limitations and lacks of this architecture mainly concern the project hypotheses like for instance:
- a node for obstacle avoidance in the environment 
- a real path planner 
- a real controller 

### Possible Improvements

Most propably the lacks of the system mentioned ahead will be implemented in the sencond assignment of the course. For the problem of the random movment while the robot's battery is low a possbile improvement consists in rememeber the perocourse movements in the environment and make the robot retrace the tree of rooms to the DOC-Station. Another possible implementation regard the urgent locations; up to now the robot prioritize urgent location but if there's more than one it decide randomly from the urgent list an idea could be assign urgent rooms a priority based on the On the time they were visited. 

--------

**Authors and Contacts**

Author: *Matteo Carlone s4652067*

to email me:  
<a href="mailto:matteo.carlone99@gmail.com" >
<img align="left" alt="Matte's mail" width="40px" src="https://user-images.githubusercontent.com/81308076/155858753-ef1238f1-5887-4e4d-9ac2-2b0bb82836e2.png" />
</a>

