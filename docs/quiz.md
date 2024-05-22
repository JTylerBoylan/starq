# STARQ Development Quiz

## General

- Create a new executable that prints out "Hello world!" to console

- Create a dummy motor controller header/class that outputs to console when a function is called

- Create a dummy leg dynamics header/class that outputs to console when the kinematics are called

- In your executable, add your dummy motor controller object and send a couple commands to it

- Add your dummy leg dynamics object

- Add a leg controller object using the dummy motor controller and leg dynamics objects

- Add a leg command publisher object using your leg controllers

- Add a trajectory file reader and a trajectory publisher

- Load and run a trajectory from a file

- Add an e-stop on the dummy motor controller class

## Robots

- Create a new executable that prints out "Hello world!" to console

- Create a dummy robot class

## STARQ


## MuJoCo

- In simulation, make the robot walk in a circle around the origin point using MPC

- Print out the frequency of the MPC to the console

- Build with the `Release` CMake flag and note the difference in frequency

## ROS2

- Create a new ROS executable that prints out "Hello world!" to console

- Run the executable using `ros2 run`

- Create a launch file that launches the `joy` node and your executable

- Create a ROS2 joystick class that loads various trajectories from a file using the buttons, and changes the frequency of the trajectory based on the joystick input