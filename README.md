# Robotic Arm Project

The goal of this project is to build a robotic arm from scratch, using learned best practices and challenging self to grow through this project.
Written in C++17 with CMake build system
Designed for extensibility and long-term robustness

The project should have an architected software with clear layer separation, and should have robust unit testing for each layer.
The architecture will include a hardware abstraction layer with polymorphic joint support: revolute, prismatic, and custom. This layer should handle configuration, construction, and initialization across hardware types:
1. PWM servos
2. Motors with position, torque, and velocity tracking via encoders and sensors
3. EtherCAT and CANBus drive support
I'd like to expand on this project by building a second project for a fully built out EtherCAT layer with dedicated EtherCAT Master.

The project will also include a modular kinematic solver supporting multiple Inverse Kinematics backends as I learn what solvers I need and how to build them.
For example:
1. Basic Analytical IK for 3-DOF arms as a starter backend
2. Jacobian IK for redundant joints, likely to be used with real hardware

The SW should have a well-structured programming hierarchy, and should have a state machine which should support:
1. Active control
2. Impedance state
3. Holding state
I'd also like to add controls for loading states, and using different GUIs with different control schemas (Cartisian, teleoperation, etc.).
