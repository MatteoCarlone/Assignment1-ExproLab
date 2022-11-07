
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

	- if the robot's battery is low but it can not reach the DOC-Station (Staring Room) it will as the battery is full

*Image of the Environment*

...........

`caption` :
	
	- Sample environment with cartesian coordinates associated with each room.

### Assumptions

## Project Structure

### Package list 

### Dependencies

## Software Components

## Launching the Sowtware
