# Robotic mail delivery project (CS225A)
Teammates: Kevin Supakkul, Andrew Brooks, Carolyn Kim

This repository contains the code for the simulation of robotic mail delivery. Video demo: https://www.youtube.com/watch?v=3RRNnR1ZjW8r

The controller code is in [project_starter_code/controller.cpp](https://github.com/kevsup/cs225a/blob/master/project_starter_code/controller.cpp)
The visualization code is in [project_starter_code/simviz.cpp](https://github.com/kevsup/cs225a/blob/master/project_starter_code/simviz.cpp)

Algorithms used: operation/task space control, posture/null space control (redundancy), force compliance control, joint space control, velocity saturation, and finite state machine.

## Diagrams and Written Report
See the written report [here.](https://github.com/kevsup/cs225a/blob/master/Mailbot_written_report.pdf)
See diagrams and slides [here.](https://github.com/kevsup/cs225a/blob/master/Mailbot_slides.pdf)

## Dependencies
The project depends on the sai2 libraries from Stanford.

## Build and run
in the main folder make a build folder and compile from there
```
mkdir build
cd build
cmake .. && make -j4
```
## run the code
go to the bin folder and then to the folder of the application you want to run.

