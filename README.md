# STARQ Documentation
*Updated 04/30/2024*

## Getting Started

### Login
```
username: nvidia
password: nvidia
```

### SSH
1. Make sure the Jetson is powered on
2. Connect to Vicon Wi-Fi
   - *Ask someone for password if needed*
3. Open Terminal (Linux) or Command Prompt (Windows)
4. SSH into the Jetson using the username and device name: $`ssh nvidia@ubuntu`
   - *Password is the same as above*
   - *If failed, test network connection using: $`ping ubuntu`*

### Building
1. Go to project workspace: $`cd ~/starq_ws`
2. Run the STARQ Docker environment: $`./run_dev.sh`
3. Build: $`colcon build`
4. Source: $`source install/setup.bash`
4. Go to the build folder: $`cd build/starq`
5. Run $`ls` to see all test executables

### VSCode in SSH
1. Open VSCode on your local computer
2. In the bottom left corner, click the `><` icon to start remote connection
3. Add SSH configuration to `ssh nvidia@ubuntu`
4. Connect using configuration and enter the password
5. `File` > `Open Folder` > `/home/nvidia/starq_ws`
   - You will have to enter the password again
6. Edit test executables located in `starq/tests`

### Powering Off from SSH
**Note: Do not power off the Jetson by removing power. This could lead to file corruption. Instead:**
1. Run $`exit` to close the Docker environment
2. While still SSHed in the Jetson, shutdown using: $`sudo shutdown -h now`
3. This should close the SSH connection as well

### SFTP
SFTP is used to copy files remotely from your local computer to the Jetson, or vice versa. 

1. Follow steps **1-3** from the **SSH** section
2. SFTP into the Jetson: $`sftp nvidia@ubuntu`
3. Use commands like `ls` and `cd` to go to the directory you want to copy to/from on the remote host
4. Use `lls` and `lcd` to do the same on your local computer
5. Copy from local to Jetson using: $`put <local_file> <remote_file>`
6. Copy from Jetson to local using: $`get <remote_file> <local_file>`
   - You can use the recursive flag (`-r`) to copy entire directories: $`put/get -r <local_dir> <remote_dir>`
7. Exit SFTP using the $`exit` command

## Other Documentation

### 1. [Hardware Information](docs/hardware.md)

### 2. [Test Executable Descriptions](docs/testing.md)

### 3. [Development Instructions](docs/dev.md)

### 4. [C++ Quick Notes](docs/cpp_notes.md)

### 5. [CAN Bus Troubleshooting](docs/can_bus.md)

### 6. [C++ Library Class Descriptions](docs/classes.md)