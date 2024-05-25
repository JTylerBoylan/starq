# STARQ Development Quiz

## Hardware

- What is the coordinate frame of the robot? Which directions are the x, y, and z axis?

- List the ODrive's in ID order based on their locations (F/B: Front/Back, L/R: Left/Right, D/I: Direct/Inverted-Drive)

- What tool would you use to edit the configuration of a ODrive?

- How do you configure a new ODrive + motor?

- What is the maximum ODrive commands that can be sent per second?

- What is the CAN socket queue length? Why is this important?

- What happens if a lot of CAN messages are sent at once with no delay?

- What are some reasons you wouldn't recieve CAN messages on the Jetson?

- Where do you go to find where the CAN pins are on the Jetson? Where is pin 1?

- What script is responsible for loading the CAN kernels on the Jetson?

- How do you turn off the Jetson? Why is it important to "soft" shutdown?

## C++

- Why are pointers used in C++?  Why do we use shared pointers over "raw" pointers?

- In the code, what are the 6 objects that we need to create a robot object with full capabilities?

- Where do you go to edit STARQ's parameters, such as link lengths, gear ratios, etc.?

# Executables

- How do you clear errors on all ODrives?

- How do you send every ODrive to its zero position?

- How do you print the info of every ODrive?

- How do you set the gains on all ODrives?

- How do you run a trajectory from the trajectories folder?

## MATLAB

- What's the format of a trajectory file?

- How do you convert a legacy trajectory file (csv) to the new format (txt)?

- How do you copy the new trajectory file onto the Jetson?

## STARQ

- Create a new executable that prints out "Hello world!" to console

- Create a STARQ Robot object and set all feet to their default positions (standing)

- Run a trajectory from the trajectories folder.  Make sure to wait for it to finish

- Wait 5 seconds, then set the forces on all legs to 10N in the -z direction

## MuJoCo

- In simulation, make the robot walk in a circle around the origin point using MPC

- Build with the `Release` CMake flag and note the difference in frequency

## ROS2

- Create a new ROS executable that prints out "Hello world!" to console

- Run the executable using `ros2 run`

- Create a launch file that launches the `joy` node and your executable

- Create a ROS2 joystick class that loads various trajectories from a file using the buttons, and changes the frequency of the trajectory based on the joystick input

- Launch using `ros2 launch`