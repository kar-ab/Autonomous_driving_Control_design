# Autonomous_driving_Control_design

This project is used to design a controller of an autonomous driving vehicle with predefined waypoints and velocity profile in Carla Simulator.


This project is part of final assignment for [Introduction to Self-Driving Cars](https://www.coursera.org/learn/intro-self-driving-cars?specialization=self-driving-cars) on [Coursera- Self Driving Specialization](https://www.coursera.org/specializations/self-driving-cars)


As part of control design, longitudinal speed  controll is done using PID control and Stanley Control used for lateral steering control.

-----

## Prerequisites 


1. Install python3.5/6 and install tkinter module 
2. Install Carla Simulator, corresponding steps can be found in [Ubuntu Guide](https://github.com/kar-ab/Autonomous_driving_Control_design/blob/master/CARLA-Setup-Guide-_Ubuntu_.pdf)
3. Clone this project contents in "PythonClient" Folder of Carla Simulator installed directory

## Run

1. Run carla server on terminal using command 
	$ cd <path_to_carla_simlator>
	$ ./CarlaUE4.sh /Game/Maps/RaceTrack -windowed -carla-server -benchmark 

2. On another terminal, run client module 

	$ python3 /Course1FinalProject/module_7.py
