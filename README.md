# Mission Coordination LAB
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)

## General info
This	project	is	to	move	three	simulated	robots	using	ROS	1	and	Gazebo from	a	starting	position	to	the	adequate	flag	(e.g., robot1	moving to	flag1,	robot2	moving to	flag2, and	robot3	moving to	flag3)	while	avoiding	any	collision	with	another	robot.
The	main	goal is	to	reach	the	adequate	flag	as	fast	as	possible.	
The	simulated	robots	are	defined	as	follow:
- An	ultrasonic	sensor	is	embedded,	with	a	limited	range	(5	meters)
- 2	motorized	wheels	allow	the	robot	to	operate	in	the	environment
- Its	current	pose	in	the	environment	is	known	(Position	and	orientation)


	
## Technologies
Project is created with:
* ROS	 – Robot	 Operating	 System https://app.theconstructsim.com/
	
## Setup
To run the project use the provided link : https://app.theconstructsim.com/l/545bf3dc/
To	run	the Gazebo simulation ,	open	a	new	terminal	and	run	the	following	instruction:
```
$ roslaunch evry_project_description simu_robot.launch
```
In	a	second	terminal,	run	the	python	script	containing	your desired	strategy (agent_timing.py or agent_poly.py)	by	executing respectively :
either :
```
$ roslaunch evry_project_strategy agent_timing.launch 
```
or :
```
$ roslaunch evry_project_strategy agent_poly.launch 
```

