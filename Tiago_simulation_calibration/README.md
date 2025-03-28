This folder contains the calibration project, trying to match the robot's simulation to the real robot.

Basically, you don't need this folder to run the simulation.
But, if you wanted to do a similar calibration for a similar robot, you can look at how we trained the S-curve model in the *base_movement_calibration* folder.
The *joints_testing* folder is actually not a real calibration folder, as we only made some tests about the joint behavior when given a JointState command on the simulation, versus a JointTrajectory command on the real robot.
We find that we can not really compare the simulation to the real robot here, because Isaac Sim lacks a JointTrajectory controller.