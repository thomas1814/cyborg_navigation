# README
This repository contains the source code for the NTNU Cyborg`s Navigation Module (ROS node).

Node name: cyborg_navigation   
Language: Python  
Numbers of actionlib server(s): 3   

## Requirements:  
* ROS   
* ROSARNL  
* ROS Move Base Message Type:

ROS move base message type can be installed:
$ sudo apt-get install ros-kinetic-move-base
  
## Features:   
* The planing state: Finds the next location. Available at actionlib server topic cyborg_navigation/planing.   
* The movinging state: The Cyborg is moving to the next location. Available at actionlib server topic cyborg_navigation/moving.   
* The talkinging state: The Cyborg is talking. Available at actionlib server topic cyborg_navigation/talking.   

Database location is at ~/navigation.db  

## Usage:
$ rosrun cyborg_navigation navigation.py
