This repo is about calibrating and testing the Tiago robot simulation.
In particular, the goal is to match the simulated Tiago movement behavior as closely as possible to the real Tiago robot.
The main difficulty encountered is to match the wheel/driving behavior. 
Since right now we have no access to the real robots' implementation of the motion controller,
we try to model the wheel acceleration behavior by fitting a function (see notebook calibration/wheel_acceleration_calibration/fit_acceleration_curve.ipynb).
We then use this fitted function inside of the simulation to simulate the velocity curve.

For more details, each of the subfolders here contains its own README.md file with a brief explanation of the subfolders contents.