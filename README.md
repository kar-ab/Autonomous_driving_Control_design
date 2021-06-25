# Vehicle Control of Self Driving Car


This project is implementation of final assignment for [Introduction to Self-Driving Cars](https://www.coursera.org/learn/intro-self-driving-cars?specialization=self-driving-cars) on [Coursera- Self Driving Specialization](https://www.coursera.org/specializations/self-driving-cars)

In this project, Vehicle Control (Longitudinal and Lateral) of Self Driving Vehicle is performed using Carla Simulator.

This goal of this project is to design a controller, which follows trajectory defined using predefined waypoints (x and y position) and velocity profile (speed at each waypoint) by providing Throttle, Brake and Steering commands to the self driving vehicle.

As part of control design, longitudinal speed control (i.e. throttle and brake commands) is performed using PID control and Stanley Control is used for lateral steering control.

-----

## Setup


1. Install Carla Simulator, corresponding steps can be found in [Ubuntu Guide](.../blob/master/CARLA-Setup-Guide-_Ubuntu_.pdf).
2. Make sure our python enviornment has tkinter module installed
3. Clone this project contents in "PythonClient" Folder of Carla Simulator directory

## Run

1. Run carla server on terminal using command 
	`
	$ cd <path_to_carla_simlator>
	$ ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark 
	`
2. On another terminal, run client module 

	`$ python3 /Course1FinalProject/module_7.py`


## Details

[racetrack_waypoints.txt](../blob/master/Course1FinalProject/racetrack_waypoints.txt): Contains list of x, y position and the respective speed to attain at each waypoint.

[controller2D.py](../blob/master/Course1FinalProject/controller2D.py): Script containing controller design

[controller_output](../blob/master/Course1FinalProject/controller_output): Folder Containing results of trajectory followed by controller

[grade_c1m7.py](../blob/master/Course1FinalProject/grade_c1m7.py): Grading script provided by Coursera for evaluation of designed Controller


## Results


## Credits: 

University of Toronto

[Coursera](https://www.coursera.org/)


