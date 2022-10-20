# Quadruped Control Library

A library to implement quadruped control based on the paper:

["Carlo, Jared &amp; Wensing, Patrick &amp; Katz, Benjamin &amp; Bledt, Gerardo &amp; Kim, Sangbae. (2018). Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control. 1-9. 10.1109/IROS.2018.8594448"](https://www.researchgate.net/publication/330591547_Dynamic_Locomotion_in_the_MIT_Cheetah_3_Through_Convex_Model-Predictive_Control)

This library's implementation is highly influenced by the [MIT Cheetah Software](https://github.com/mit-biomimetics/Cheetah-Software). The main purpose of the library is to allow it to be easily used in ROS1 and ROS2 applications or any simulation engine of choice. This library is also written to be more readable and allows an easier understanding of MPC-controlled quadruped approach. The code is also written to support more types of robot other than classic quadrupedal robots, such as:

- Bipedal robots
- Hexapod robots

The planned components of the library are:

- Control components
    - [x] convex Model Predictive Control
    - [ ] Leg Controller 
- Foot movement components
    - [x] Fixed Gait
    - [x] Fixed Foot Trajectory 
    - [ ] Fixed Foot Planner

## Requirements
- [qpOASES](https://github.com/coin-or/qpOASES)