# Running STARQ

### SSH
1. Make sure the Jetson is powered on
2. Connect to Vicon Wi-Fi
   - *Ask someone for password if needed*
3. Open Terminal (Linux) or Command Prompt (Windows)
4. SSH into the Jetson using the username and device name: $`ssh nvidia@ubuntu`
   - *If failed, test network connection using: $`ping ubuntu`*
   - *If you recieve a ping response, retry logging in*
   - *Otherwise, check WiFi settings to make sure you're on the same network as the Jetson*
```
username: nvidia
password: nvidia
```

### Check for Git Updates
1. Go to project workspace: $`cd ~/starq_ws/src`
2. Git pull: $`git pull`
   - *If you get a merge error, rename/delete changed files and try again.*
3. Go to ROS workspace: $`cd ~/starq_ws`
4. Rebuild: $`colcon build`

### Utility Commands
- Utility commands are for running simple STARQ commands and procedures from the Terminal
- Includes clearing errors, setting gains, running trajectories, and more
- Full list is found [here](1b_UtilCommands.md)

### Test Executables
- Test executables are for running unit tests for various components
- Full list is found [here](1c_TestExecutables.md)

### VSCode in SSH
VSCode can be used to open the project workspace in your editor through a SSH connection
1. Open VSCode on your local computer
2. In the bottom left corner, click the `><` icon to see remote connection settings
3. Add SSH configuration for `ssh nvidia@ubuntu`
4. Open settings again, Connect to Host, and enter password
5. `File` > `Open Folder` > `/home/nvidia/starq_ws`
   - You will have to enter the password again
6. Utility commands located in `starq/utils`
7. Test executables located in `starq/tests`

### Rebuilding
If you make edits to the code, you will need to rebuild
1. Go to project workspace: $`cd ~/starq_ws`
2. Build: $`colcon build`
3. Source: $`source install/setup.bash` *(Only if editing launch files)*

### Upload changes to Git
If you make changes that you want to keep, you'll want to upload your changes to git
1. Make sure you're up-to-date: $`git pull`
2. Use $`git status` to see your modified files
3. Add all modified files: $`git add -A`
4. Write your commit message: $`git commit -m "my update"`
5. Push changes: $`git push origin main`

### Powering Off from SSH
**Note: Do not power off the Jetson by removing power. This could lead to file corruption. Instead:**
1. While still SSHed in the Jetson, shutdown using: $`sudo shutdown -h now`
2. This should close the SSH connection as well

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