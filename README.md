# pololu-3pi-robotics
Arduino C++ code for a Pololu 3pi+ robot simulating Roomba-like behavior. Features PID/PD control, odometry, sonar sensors, an FSA for behavior switching. Built for SDSU Robotics Labs.

This repository contains the code and documentation for my Robotics Labs project at SDSU. It features embedded programming for the Pololu 3pi+ robot using Arduino C++, focusing on real-time control and autonomous navigation. Key components include:
PID and PD control for precise motor regulation
Odometry and sonar sensor integration for environment awareness
Finite State Automaton (FSA) to manage behavior transitions
Particle Filter Localization to estimate robot position on a grid
The system simulates Roomba-like behavior, adapting dynamically to obstacles and changing conditions. All controller tuning and logic were implemented through iterative testing and experimentation in real-world conditions.
