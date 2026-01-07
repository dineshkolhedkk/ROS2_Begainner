# ROS 2 and Linux Command 

List of essential Linux and ROS 2 commands for development, debugging, and workspace management.
___
## 1. Linux Terminal Essentials

These commands help you navigate the system and manage files.

* `pwd` : **Print Working Directory** - Shows the full path of your current folder.
* `ls` : **List** - Lists files and folders in the current directory.
* `ls -a` : **List All** - Shows hidden files (files starting with a dot).
* `cd <path>` : **Change Directory** - Move to a specific folder (e.g., `cd ~/ros2_ws`).
* `cd ..` : **Move Up** - Goes to the parent directory.
* `mkdir -p <name>` : **Make Directory** - Creates a new folder. The `-p` flag creates parent folders if they don't exist.
* `touch <file>` : **Create File** - Creates a new empty file.
* `rm -rf <folder>` : **Remove Recursive Force** - Deletes a folder and everything inside it (commonly used to clear `build` and `install` folders).
* `cp -r <source> <dest>` : **Copy** - Copies files or folders.
* `mv <old_name> <new_name>` : **Move/Rename** - Moves a file or renames it.
* `chmod +x <file>` : **Change Mode** - Makes a script executable (required for Python nodes).
* `sudo apt update` : **Update Packages** - Updates the list of available software.
* `htop` : **Interactive Process Viewer** - Shows CPU and RAM usage (useful to see if ROS is lagging).

## 2. ROS 2 Environment & Workspace

Commands for building and configuring your ROS 2 environment.

* `source /opt/ros/<distro>/setup.bash` : **Source ROS 2** - Loads the ROS 2 environment into your terminal (e.g, `source /opt/ros/humble/setup.bash`.
* `colcon build` : **Build Workspace** - Compiles all packages in your `src` folder.
* `colcon build --symlink-install` : **Symlink Build** - Recommended for Python; allows you to see code changes without rebuilding.
* `colcon build --packages-select <pkg>` : **Selected Build** - Only builds the package you specify.
* `source install/setup.bash` : **Source Workspace** - Makes your custom packages available to run in the current terminal.
* `ros2 doctor` : **Health Check** - Scans your ROS 2 setup for errors or network issues.

## 3. ROS 2 Package Management

Commands to create and explore packages.

* `ros2 pkg list` : Lists every ROS 2 package installed on your system.
* `ros2 pkg create --build-type <type> <name>` : Creates a new package (Types: `ament_cmake` or `ament_python`).
* `ros2 pkg executables <pkg>` : Lists the nodes available inside a specific package.
* `ros2 pkg prefix <pkg>` : Shows the installation path of a package.

## 4. Running ROS 2 Nodes

Commands to execute your code.

* `ros2 run <pkg> <executable>` : Starts a single ROS 2 node.
* `ros2 launch <pkg> <launch_file>` : Runs a launch file (starts multiple nodes and configurations).
* `ros2 node list` : Shows all active nodes currently running.
* `ros2 node info /<node_name>` : Shows details about a node (publishers, subscribers, services).

## 5. Introspection (Topics, Services, Actions)

Commands to see data flowing through the system.

* `ros2 topic list` : Lists all active communication streams.
* `ros2 topic echo /<topic>` : Prints live data from a topic to the terminal.
* `ros2 topic info /<topic>` : Shows the message type and number of nodes connected to the topic.
* `ros2 topic hz /<topic>` : Shows the frequency (speed) at which messages are published.
* `ros2 interface show <msg_type>` : Shows the structure of a specific message type.
* `ros2 service list` : Lists all available services.
* `ros2 param list` : Lists parameters for all active nodes.

---
# 🚀 Ultimate ROS 2 & Linux Developer Reference

This guide covers the most important commands for robotics development in ROS 2.

## 📁 1. Advanced File & Disk Management

| Command | Function |
| --- | --- |
| `du -sh *` | Check the size of all folders in the current directory. |
| `df -h` | Show free disk space on the system. |
| `find . -name "*.py"` | Find all Python files in the current workspace. |
| `grep -r "Node" .` | Search for the word "Node" inside all files in the directory. |
| `ln -s <target> <link>` | Create a symbolic link (shortcut) to a file/folder. |
| `tar -czvf ws.tar.gz src/` | Compress your `src` folder into a `.tar.gz` file. |
| `watch -n 1 ls` | Run `ls` every second (useful to watch files being created). |

## 📦 2. ROS 2 Package & Build (Colcon)

| Command | Function |
| --- | --- |
| `ros2 pkg list` | List all installed ROS 2 packages. |
| `ros2 pkg executables <pkg>` | See all runnable nodes in a package. |
| `colcon build --symlink-install` | Build workspace and link files (no need to rebuild for Python changes). |
| `colcon build --packages-up-to <pkg>` | Build a specific package and all its dependencies. |
| `colcon test` | Run all tests for the packages in the workspace. |
| `rosdep install --from-paths src -y` | Install all missing dependencies for your custom packages automatically. |

## 📡 3. Communication Introspection (Topics & Services)

| Command | Function |
| --- | --- |
| `ros2 topic list -t` | List all topics along with their message types. |
| `ros2 topic hz /cmd_vel` | Check the publishing frequency of a topic. |
| `ros2 topic bw /image_raw` | Check the bandwidth (data usage) of a topic. |
| `ros2 topic pub -r 10 /topic <type> "{data}"` | Publish a message at 10Hz. |
| `ros2 service list -t` | List all services and their types. |
| `ros2 service call /reset std_srvs/srv/Empty` | Manually trigger a service. |
| `ros2 interface show <type>` | See the fields inside a message, service, or action. |

## 🤖 4. Node & Parameter Control

| Command | Function |
| --- | --- |
| `ros2 node list` | Show all active nodes. |
| `ros2 node info /my_node` | Show publishers, subscribers, and services for a node. |
| `ros2 param list` | List all parameters for all running nodes. |
| `ros2 param get /node param_name` | Read a specific parameter value. |
| `ros2 param set /node param_name value` | Change a parameter value on the fly. |
| `ros2 param dump /node > file.yaml` | Save a node's parameters to a file. |

## 🎥 5. Actions & Recording (Bag Files)

| Command | Function |
| --- | --- |
| `ros2 action list` | List all available actions. |
| `ros2 action info /navigate_to_pose` | See which nodes are the server and client. |
| `ros2 bag record -a` | Record every topic currently running into a bag file. |
| `ros2 bag record /topic1 /topic2` | Record specific topics. |
| `ros2 bag play <file_name>` | Play back recorded data. |
| `ros2 bag info <file_name>` | See duration, message count, and topics in a bag. |

## 🛠️ 6. System & Debugging Tools

| Command | Function |
| --- | --- |
| `rqt_graph` | Open a visual map of nodes and topics. |
| `rqt_console` | Open a GUI to see all ROS log messages (Debug, Info, Warn, Error). |
| `ros2 doctor` | Check your system for network or setup issues. |
| `ros2 daemon stop; ros2 daemon start` | Restart the ROS 2 discovery background process. |
| `printenv | grep ROS` |
| `ip addr show` | Check your IP address (needed for multi-robot setup). |

## 🐍 7. Python & Environment

| Command | Function |
| --- | --- |
| `python3 -m pip install <library>` | Install Python libraries needed for your nodes. |
| `which python3` | See which Python interpreter ROS is using. |
| `export ROS_DOMAIN_ID=42` | Change the domain ID to isolate your robot on a network. |
| `source ~/.bashrc` | Refresh your terminal settings after editing the bashrc file. |


## 🛠 8. Advanced Linux & Permissions

| Command | Function |
| --- | --- |
| `sudo chown -R $USER:$USER ~/ros2_ws` | Changes ownership of your workspace to your current user. |
| `ls -l` | Shows file permissions (read, write, execute). |
| `groups` | Shows which groups your user belongs to (important for `dialout`/serial access). |
| `sudo usermod -aG dialout $USER` | Adds your user to the serial port group (needed for LiDAR/Arduino). |
| `grep -i "error" /var/log/syslog` | Searches system logs for hardware or driver errors. |
| `alias` | Lists all your custom terminal shortcuts. |
| `history | grep "ros2"` |

## 🌐 9. Networking & Multi-Robot Setup

When running ROS 2 across multiple computers, these commands are vital.

| Command | Function |
| --- | --- |
| `ping <ip_address>` | Checks if another computer/robot is reachable on the network. |
| `ssh username@robot.local` | Logs into your robot's computer remotely. |
| `scp <file> user@robot:/path` | Securely copies a file from your PC to the robot. |
| `nmap -sn 192.168.1.0/24` | Scans the local network for all connected devices. |
| `export ROS_LOCALHOST_ONLY=1` | Restricts ROS 2 traffic to your computer only (prevents interference). |
| `ros2 multicast send` | Tests if your network supports multicast (needed for discovery). |
| `ros2 multicast receive` | Listens for the multicast test signal. |

## 🐙 10. Git for ROS 2 Development

Essential for managing your code on GitHub.

| Command | Function |
| --- | --- |
| `git init` | Initializes a new repository in your `src` folder. |
| `git clone <url>` | Downloads an existing ROS 2 package from GitHub. |
| `git status` | Shows which files have been modified. |
| `git add .` | Stages all changes for a commit. |
| `git commit -m "Message"` | Saves your changes locally. |
| `git push origin main` | Uploads your code to GitHub. |
| `git pull` | Downloads the latest version of the code from GitHub. |
| `git checkout -b <branch>` | Creates a new branch for a specific feature (e.g., `git checkout -b lidar-fix`). |

## 📦 11. VCS (Version Control System) Tools

ROS 2 developers use `vcs` to manage workspaces with dozens of different repositories.

| Command | Function |
| --- | --- |
| `vcs import src < my_robot.repos` | Clones all repositories listed in a `.repos` file into `src`. |
| `vcs export src > backup.repos` | Saves a list of all current repos and branches to a file. |
| `vcs status src` | Checks the git status of every package in the `src` folder at once. |
| `vcs pull src` | Updates every single package in your workspace at once. |

## 📊 12. Troubleshooting & Performance

| Command | Function |
| --- | --- |
| `ros2 node info /node_name` | Identifies which topics a node is dropping or missing. |
| `ros2 lifecycle list` | Lists nodes using the Managed Lifecycle system. |
| `rqt_graph` | Visualizes the computation graph to find "broken" connections. |
| `rqt_plot` | Graphs numeric data (like sensor values) in real-time. |
| `vmstat 1` | Reports virtual memory statistics every second. |
| `iostat` | Checks if your disk (SD card on Raspberry Pi) is the bottleneck. |


## 🏗 13. Robot Modeling (URDF & Xacro)

When building the physical description of your robot, these commands are essential for debugging transforms ().

| Command | Function |
| --- | --- |
| `xacro robot.urdf.xacro > robot.urdf` | Converts a Xacro file into a plain URDF file. |
| `check_urdf robot.urdf` | Verifies the syntax and links of your URDF file. |
| `ros2 run tf2_tools view_frames` | Generates a PDF showing the entire transform tree (). |
| `ros2 run tf2_ros tf2_echo <frame_a> <frame_b>` | Prints the specific translation and rotation between two links. |
| `ros2 run rviz2 rviz2` | Opens the 3D visualization tool to see your robot model. |

## 🔄 14. Managed Lifecycle Nodes

Some advanced nodes use a "State Machine" to ensure they start in the correct order.

| Command | Function |
| --- | --- |
| `ros2 lifecycle nodes` | Lists all nodes that support lifecycle management. |
| `ros2 lifecycle get /node_name` | Shows the current state (e.g., `unconfigured`, `inactive`, `active`). |
| `ros2 lifecycle set /node_name configure` | Transitions a node from `unconfigured` to `inactive`. |
| `ros2 lifecycle set /node_name activate` | Starts the node's processing (moves to `active`). |

## 🚀 15. Advanced Launch Commands

Launch files in ROS 2 are usually Python scripts. These commands help you debug them.

| Command | Function |
| --- | --- |
| `ros2 launch <pkg> <file> --show-args` | Shows all arguments/parameters you can pass to the launch file. |
| `ros2 launch <pkg> <file> arg_name:=value` | Runs a launch file with custom argument values. |
| `ros2 launch <pkg> <file> --debug` | Runs the launch system with detailed debug output. |

## 🛰 16. DDS & RMW (Middleware Settings)

ROS 2 uses DDS for communication. Sometimes you need to tweak the "plumbing."

| Command | Function |
| --- | --- |
| `ros2 doctor --report` | Generates a massive report of your network and middleware settings. |
| `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Switches the middleware to CycloneDDS (often used for stability). |
| `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Switches to FastDDS (the default in many distros). |
| `ros2 security list_keys` | Lists security keys if you are using SROS (Secure ROS). |

## 💻 17. Linux Process & Performance Tuning

For high-performance robotics, you need to manage your CPU priority.

| Command | Function |
| --- | --- |
| `nice -n -20 ros2 run <pkg> <node>` | Runs a node with the highest possible CPU priority. |
| `ionice -c 1 -n 0 ros2 bag record -a` | Gives the bag recorder "Real-time" disk priority. |
| `lscpu` | See how many cores your robot has (crucial for multi-threading). |
| `free -m` | Checks RAM usage in Megabytes. |
| `killall -9 <node_name>` | Instantly kills all processes matching that name. |

To wrap up your GitHub reference, we will cover **Simulation (Gazebo)**, **Visualization (RViz2)**, **Parameter files (YAML)**, and **Logging**. This ensures your documentation covers everything from writing code to testing it in a virtual world.

---

## 🌍 17. Simulation & Gazebo (Ignition/Classic)

When testing robots without hardware, you will use Gazebo.

| Command | Function |
| --- | --- |
| `ros2 launch gazebo_ros gazebo.launch.py` | Starts the Gazebo simulator. |
| `ros2 run gazebo_ros spawn_entity.py -file <urdf> -entity <name>` | Manually spawns a robot model into the simulation. |
| `ign gazebo <world_file>` | Starts Ignition Gazebo (used in newer ROS 2 versions like Humble/Rolling). |
| `ros2 topic echo /model/<name>/odometry` | Checks if the simulation is actually publishing robot data. |

## 👁️ 18. Visualization (RViz2)

RViz2 is the "eyes" of the robot, showing what it sees (LiDAR, Cameras, TF).

| Command | Function |
| --- | --- |
| `rviz2` | Opens the main visualization window. |
| `rviz2 -d <config_file.rviz>` | Opens RViz with a saved layout (saves time). |
| `ros2 run tf2_tools view_frames` | Generates a PDF called `frames.pdf` to visualize your TF tree. |
| `ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent child` | Manually creates a connection between two robot parts. |

## 📝 19. Parameters & YAML Configuration

Parameters allow you to change robot settings without recompiling code.

| Command | Function |
| --- | --- |
| `ros2 param list` | Shows all parameters currently loaded in all nodes. |
| `ros2 param describe <node> <param>` | Gives the description and constraints of a parameter. |
| `ros2 param set <node> <param> <value>` | Updates a value while the robot is running. |
| `ros2 run <pkg> <executable> --ros-args --params-file <file.yaml>` | Starts a node using settings from a YAML file. |

## 📜 20. Logging & Console Output

When things go wrong, you need to see the "hidden" messages from your code.

| Command | Function |
| --- | --- |
| `ros2 run <pkg> <exe> --ros-args --log-level debug` | Forces a node to show every single "Debug" message. |
| `ls ~/.ros/log` | Shows where the physical log files are stored on your hard drive. |
| `rqt_console` | A GUI tool that filters and sorts all error messages from all nodes. |
| `tail -f ~/.ros/log/latest/launch.log` | Shows the live log output of your latest launch file. |

## 🛠 21. Hardware Interfaces (Serial/I2C/CAN)

Commands for connecting your Linux PC to physical sensors and motors.

| Command | Function |
| --- | --- |
| `lsusb` | Lists all USB devices (check if your LiDAR/Camera is plugged in). |
| `dmesg -w` | Watches system messages (useful to see if a device is disconnecting). |
| `i2cdetect -y 1` | Checks for sensors connected to the I2C bus (on Raspberry Pi/Jetson). |
| `candump can0` | Shows raw data from a CAN bus (used in heavy-duty industrial robots). |


## 🧭 22. Navigation2 (Nav2) & SLAM

Nav2 is the brain for path planning, and SLAM is used for creating maps.

| Command | Function |
| --- | --- |
| `ros2 launch nav2_bringup tb3_simulation_launch.py` | Starts the Nav2 stack (standard example). |
| `ros2 run nav2_map_server map_saver_cli -f ~/my_map` | Saves the currently generated map to your disk. |
| `ros2 lifecycle set /map_server configure` | Manually configures the map server if it hangs. |
| `ros2 topic pub /goal_pose ...` | Manually sends a coordinate to the robot to move to. |
| `ros2 launch slam_toolbox online_async_launch.py` | Starts SLAM to begin building a 2D map. |

## 📶 23. Quality of Service (QoS)

QoS settings determine how data is handled (e.g., "Best Effort" for sensor data vs. "Reliable" for commands).

| Command | Function |
| --- | --- |
| `ros2 topic list -v` | Shows the QoS profile (Reliability, Durability) for all topics. |
| `ros2 run topic_monitor topic_monitor` | Monitors the health and latency of data streams. |
| `ros2 run tf2_ros tf2_monitor` | Checks the latency and frequency of transform updates (). |

## 🏗️ 24. Professional Build & Clean

When your project gets large, standard `colcon build` can be slow or messy.

| Command | Function |
| --- | --- |
| `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | Builds with optimizations (makes code run faster). |
| `colcon build --continue-on-error` | Keeps building other packages even if one fails. |
| `colcon list` | Shows a list of all packages found in the workspace. |
| `colcon graph` | Shows the dependency tree of your packages in the terminal. |

## 📦 25. Deployment & Production

Commands for when you are ready to put your robot "in the field."

| Command | Function |
| --- | --- |
| `ros2 run ros2cli_test_interfaces <args>` | Checks if your custom messages are properly installed system-wide. |
| `ros2 run swri_console swri_console` | An advanced alternative to `rqt_console` for large log files. |
| `ros2 pkg xml <pkg>` | Prints the `package.xml` to check version and author info. |
| `journalctl -u my_robot_service -f` | Watches the logs if you have set ROS 2 to start as a Linux Service. |


## 🏗️ 27. ROS 2 Component Containers

Components allow multiple nodes to run in a single process to reduce overhead and latency.

| Command | Function |
| --- | --- |
| `ros2 component list` | Lists all active component containers and loaded components. |
| `ros2 component types` | Lists all component classes available in your installed packages. |
| `ros2 run rclcpp_components component_container` | Starts an empty container to host components. |
| `ros2 component load /container <pkg> <plugin_name>` | Dynamically loads a node into a running container. |
| `ros2 component unload /container <id>` | Removes a specific component from a container. |

## 🔐 28. SROS2 (Security & Encryption)

Commands for securing your robot's communication using DDS-Security.

| Command | Function |
| --- | --- |
| `ros2 security create_keystore ~/my_keystore` | Initializes a folder to store security certificates. |
| `ros2 security create_enclave ~/my_keystore /my_node` | Creates security credentials for a specific node. |
| `ros2 security generate_artifacts` | Generates the XML/Signed files needed for secure communication. |
| `export ROS_SECURITY_ENABLE=true` | Forces ROS 2 to only allow encrypted communication. |
| `export ROS_SECURITY_STRATEGY=Enforce` | Blocks any node that doesn't have valid security keys. |

## ⏱️ 29. Performance Profiling & Benchmarking

Essential for optimizing high-speed robots or vision processing.

| Command | Function |
| --- | --- |
| `top -p $(pgrep -d',' -f ros2)` | Monitors CPU/Memory for only your ROS-related processes. |
| `valgrind --tool=callgrind ros2 run <pkg> <node>` | Profiles C++ nodes to find which functions are slow. |
| `ros2 run tracetools status` | Checks if LTTng tracing is enabled for performance analysis. |
| `ros2 run ros2_tracing trace_example` | Captures high-frequency internal ROS 2 events. |

## 🧩 30. Advanced Topic & Interface Filtering

| Command | Function |
| --- | --- |
| `ros2 topic list --no-daemon` | Lists topics without using the discovery background process. |
| `ros2 interface packages` | Lists all packages that contain message/service definitions. |
| `ros2 interface proto <type>` | Prints an example of how to format a message in YAML. |
| `ros2 topic echo --once /topic` | Prints only one message and then exits automatically. |
| `ros2 topic echo --field data /topic` | Prints only a specific field of a message instead of the whole thing. |

## 🐧 31. Low-Level Linux Hardware Debugging

Commands for when your sensors (USB/Serial) act up.

| Command | Function |
| --- | --- |
| `udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0)` | Gets unique ID of a USB device for `udev` rules. |
| `stty -F /dev/ttyUSB0 speed 115200` | Manually sets the baud rate of a serial port. |
| `ls -l /dev/serial/by-id/` | Shows human-readable names for connected microcontrollers. |
| `v4l2-ctl --list-devices` | Lists all connected USB cameras. |
| `v4l2-ctl -d /dev/video0 --list-formats-ext` | Shows supported resolutions and frame rates of a camera. |

## 🛠️ 32. Bash Automation for ROS 2

Adding these to your `~/.bashrc` will save hours of typing.

| Alias/Function | Purpose |
| --- | --- |
| `alias ccb='colcon build --symlink-install'` | Build shortcut. |
| `alias sds='source install/setup.bash'` | Quick workspace source. |
| `alias rt='ros2 topic list'` | Fast topic check. |
| `alias rn='ros2 node list'` | Fast node check. |


## 💾 33. Advanced ROS 2 Bag (Data Logging)

Beyond simple recording, these commands allow you to filter and manipulate data.

| Command | Function |
| --- | --- |
| `ros2 bag reindex <bag_folder>` | Repairs a corrupted bag file if it didn't close properly. |
| `ros2 bag convert -i <input_bag> -o <output_bag>` | Changes the storage format or compression of a bag. |
| `ros2 bag play <bag> --remap /old_topic:=/new_topic` | Plays data back but changes the topic name. |
| `ros2 bag play <bag> -r 2.0` | Plays the recording back at 2x speed. |
| `ros2 bag play <bag> --loop` | Automatically restarts the recording when it finishes. |

## ⚡ 34. DDS Tuning & Environment Variables

DDS (Data Distribution Service) is the "middleware." These variables change how ROS 2 behaves on a network.

| Variable | Function |
| --- | --- |
| `export ROS_DOMAIN_ID=X` | Groups robots on the same network (0-232). Robots with different IDs cannot talk. |
| `export ROS_LOCALHOST_ONLY=1` | Limits ROS 2 to internal communication (no network traffic). |
| `export FASTRTPS_DEFAULT_PROFILES_FILE=<path>` | Loads a custom XML for advanced DDS tuning. |
| `export ROS_PYTHON_VERSION=3` | Forces the system to use Python 3 for all ROS scripts. |
| `export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"` | Customizes how logs look in your terminal. |

## 🎮 35. Gazebo Plugins & Controls

Commands for interacting with the physics engine programmatically.

| Command | Function |
| --- | --- |
| `ros2 service call /gazebo/pause_physics std_srvs/srv/Empty` | Pauses time in the simulation. |
| `ros2 service call /gazebo/unpause_physics std_srvs/srv/Empty` | Resumes simulation time. |
| `ros2 run gazebo_ros spawn_entity.py -h` | Shows all options for spawning robots (position, orientation, etc.). |
| `gz topic -l` | Lists internal Gazebo topics (different from ROS topics). |

## 🛠️ 36. Advanced Colcon Build Flags

For complex workspaces with many developers.

| Command | Function |
| --- | --- |
| `colcon build --parallel-workers <num>` | Limits CPU cores used (useful for low-power PCs like Raspberry Pi). |
| `colcon build --cmake-clean-cache` | Clears the CMake cache without deleting the whole folder. |
| `colcon build --event-handlers console_direct+` | Shows real-time compiler warnings/errors instead of hiding them. |
| `colcon build --base-paths <path>` | Builds a workspace located in a different directory. |
| `colcon build --mixin release` | Uses a predefined "release" configuration for maximum speed. |

## 🧪 37. Unit Testing Commands

| Command | Function |
| --- | --- |
| `colcon test --packages-select <pkg>` | Runs tests only for the specific package. |
| `colcon test-result --all` | Shows a detailed summary of all passed/failed tests. |
| `colcon test-result --verbose` | Shows the exact error message for failed code tests. |

## 🖥️ 38. System-Level Linux Monitoring

| Command | Function |
| --- | --- |
| `watch -n 0.1 "ros2 topic hz /topic"` | Monitors frequency with a 0.1s refresh rate. |
| `lsof -i :11811` | Checks if the ROS 2 Discovery Server port is being used. |
| `nmcli device wifi list` | Checks signal strength of the robot's Wi-Fi. |
| `systemctl list-units --type=service | grep ros` |


## 📸 39. Visual SLAM & Computer Vision

For robots using cameras (RGB-D or Stereo) instead of LiDAR.

| Command | Function |
| --- | --- |
| `ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py` | Starts NVIDIA’s hardware-accelerated VSLAM. |
| `ros2 run image_view image_view --ros-args -r image:=/camera/image_raw` | Simple GUI to view a raw camera feed. |
| `ros2 topic bw /camera/image_raw` | Measures the bandwidth (crucial for high-res video feeds). |
| `ros2 run v4l2_camera v4l2_camera_node` | Starts a standard USB camera node. |

## 🐳 40. Docker for ROS 2

Modern robotics uses Docker to ensure code runs the same on every robot.

| Command | Function |
| --- | --- |
| `docker pull ros:humble` | Downloads the official ROS 2 Humble image. |
| `docker run -it --net=host ros:humble` | Starts a ROS 2 container sharing the host network (needed for DDS). |
| `docker exec -it <container_id> bash` | Opens a second terminal inside a running ROS container. |
| `docker build -t my_robot_image .` | Builds a custom image from your `Dockerfile`. |
| `docker-compose up` | Starts multiple containers (e.g., Robot + Simulation + Database). |

## 📟 41. Micro-ROS (Microcontrollers)

For running ROS 2 on ESP32, Arduino, or STM32.

| Command | Function |
| --- | --- |
| `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0` | Starts the bridge between the microcontroller and ROS 2. |
| `ros2 run micro_ros_setup create_firmware_ws.sh` | Sets up the build environment for micro-ROS firmware. |
| `ros2 run micro_ros_setup build_firmware.sh` | Compiles the code for the specific microcontroller. |
| `ros2 run micro_ros_setup flash_firmware.sh` | Uploads the code to the hardware. |

## 🛠️ 42. Quality Assurance & Linting

Professional code must be clean. These commands check your code style.

| Command | Function |
| --- | --- |
| `ament_cpplint <file>` | Checks C++ code against the ROS 2 style guide. |
| `ament_flake8 <file>` | Checks Python code for PEP8 compliance and errors. |
| `ament_xmllint <file>` | Checks `package.xml` or launch files for syntax errors. |
| `colcon build --cmake-args -DSTOP_ON_WARNING=ON` | Forces the build to fail if there are any code warnings. |

## 🚀 43. System Level "Nuclear" Options

When the workspace is completely broken and nothing else works.

| Command | Function |
| --- | --- |
| `rm -rf build/ install/ log/` | The "Triple Wipe" – cleans all build artifacts. |
| `killall -9 -u $USER` | Kills every single process owned by you (useful if ROS freezes the UI). |
| `sudo ldconfig` | Refreshes the system's library links (fixes "library not found" errors). |
| `ccache -C` | Clears the compiler cache if you suspect corrupted builds. |



## 🌉 44. ROS 2 - Gazebo Transport Bridge

Used for the newer "Ignition" or "Gazebo Sim" versions to translate messages.

| Command | Function |
| --- | --- |
| `ros2 run ros_gz_bridge parameter_bridge /topic@msg@gz_msg` | Bridges a specific topic between ROS 2 and Gazebo. |
| `ros2 run ros_gz_image image_bridge /camera_raw` | Specialized high-performance bridge for camera data. |
| `ign service -l` | Lists internal Gazebo Sim services. |
| `ign model --info -m <robot_name>` | Shows the physical properties (mass, friction) of a model in sim. |

## 📡 45. Advanced DDS Troubleshooting (RTI & FastDDS)

When the network is complex, you need to look at the "wires."

| Command | Function |
| --- | --- |
| `ros2 run fastdds_statistics_tool visualization` | Monitors network traffic and latency for FastDDS. |
| `rtiddsspy` | (If using RTI Connext) A powerful tool to see raw packets on the wire. |
| `ros2 run rclcpp_components component_container_isolated` | Runs components in their own threads to prevent one node from crashing the rest. |
| `export FASTRTPS_DEFAULT_PROFILES_FILE=my_dds_config.xml` | Forces a specific network configuration. |

## 📐 46. Coordinate Transforms (TF2) - Pro Level

| Command | Function |
| --- | --- |
| `ros2 run tf2_ros tf2_monitor <frame_1> <frame_2>` | Checks the delay (latency) and jitter of a specific link. |
| `ros2 run tf2_tools view_frames` | Generates a PDF of the robot's physical structure. |
| `ros2 run tf2_ros static_transform_publisher --x 1 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id world --child-frame-id map` | Manually links two frames that are not connected. |

## 👨‍💻 47. Linux Developer Productivity

Commands to help you manage multiple terminals and code efficiently.

| Command | Function |
| --- | --- |
| `tmux` | Starts a terminal multiplexer (run multiple ROS nodes in one window). |
| `terminator` | A terminal emulator that allows easy splitting of screens. |
| `grep -r "TODO" src/` | Finds all your "to-do" notes in your source code. |
| `find src -name "CMakeLists.txt" -exec grep -l "executable" {} +` | Lists all packages that create an executable. |
| `du -h --max-depth=1 | sort -hr` |

## 🛡️ 48. Security Enclave Management

| Command | Function |
| --- | --- |
| `ros2 security create_key <keystore> <name>` | Generates a specific key for a secure node. |
| `ros2 security list_enclaves <keystore>` | Lists all secured nodes in your system. |
| `ros2 security get_enclave <node_name>` | Shows the security status of a specific node. |

## 📦 49. ROS 2 "Distro" & Version Management

| Command | Function |
| --- | --- |
| `ros2 --help` | Shows the main help menu for all ros2 commands. |
| `ros2 pkg info <package_name>` | Shows the version, maintainer, and website for a package. |
| `ls /opt/ros/` | Shows which ROS versions (Humble, Foxy, etc.) are installed on your system. |
| `apt-cache policy ros-humble-desktop` | Checks if a ROS 2 update is available. |

## 🧹 50. The "Clean Everything" Script

If you want to provide a "Master Reset" command for your users, they can run this to fix almost any environment error:

```bash
# Clean Workspace Script
rm -rf build/ install/ log/ && \
colcon build --symlink-install && \
source install/setup.bash && \
ros2 daemon stop && \
ros2 daemon start

```

### 📦 Box 51: Advanced `colcon` Build Optimization

| Command | Function |
| --- | --- |
| `colcon build --executor sequential` | Builds one package at a time (fixes RAM crashes on Raspberry Pi). |
| `colcon build --event-handlers desktop_notification+` | Sends a Linux system notification when the build finishes. |
| `colcon build --cmake-args -Wno-dev` | Suppresses CMake developer warnings for a cleaner output. |
| `colcon build --base-paths src/subfolder` | Builds only a sub-directory of your source folder. |

### 🔍 Box 52: Deep Topic Inspection

| Command | Function |
| --- | --- |
| `ros2 topic echo --csv /topic` | Prints topic data in CSV format (perfect for copy-pasting into Excel). |
| `ros2 topic echo --filter "m.data > 10"` | (Advanced) Only shows messages where the data meets a condition. |
| `ros2 topic pub --once /topic <type> <data>` | Publishes a single message and immediately stops. |
| `ros2 topic pub --print 10 /topic <type>` | Prints the message being published to the console while sending. |

### 🐢 Box 53: Turtlesim Testing Sandbox

| Command | Function |
| --- | --- |
| `ros2 run turtlesim turtlesim_node` | Starts the standard ROS 2 simulator for practice. |
| `ros2 run turtlesim turtle_teleop_key` | Starts keyboard control for the turtle. |
| `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, name: 'turtle2'}"` | Creates a second turtle programmatically. |
| `ros2 param set /turtlesim background_r 255` | Changes the simulator background color via parameters. |

### 🛠️ Box 54: Interface (Msg/Srv) Discovery

| Command | Function |
| --- | --- |
| `ros2 interface list` | Shows every message, service, and action available on the system. |
| `ros2 interface list -m` | Lists only Messages. |
| `ros2 interface list -s` | Lists only Services. |
| `ros2 interface package <pkg_name>` | Shows all interfaces defined within a specific package. |

### 💾 Box 55: Bag File Filtering & Splitting

| Command | Function |
| --- | --- |
| `ros2 bag record -b 1048576` | Sets the internal buffer size (useful for high-speed camera data). |
| `ros2 bag record --max-bag-size 1000000` | Automatically splits the recording into 1MB files. |
| `ros2 bag record -o custom_name /topic` | Saves the recording to a specific folder name. |
| `ros2 bag record --exclude "/hidden/*"` | Records everything *except* topics matching the pattern. |

### 🌐 Box 56: Network Troubleshooting (IP/DDS)

| Command | Function |
| --- | --- |
| `hostname -I` | Quickly displays the IP address of your robot. |
| `ros2 run demo_nodes_cpp talker` | Standard test to see if ROS 2 communication is working. |
| `ros2 run demo_nodes_py listener` | Complements the talker to verify cross-language communication. |
| `export ROS_DISCOVERY_SERVER=192.168.1.50:11811` | Connects to a specific Discovery Server for large-scale networks. |

### 🐍 Box 57: Python Development Helpers

| Command | Function |
| --- | --- |
| `pip3 list | grep rclpy` |
| `python3 -c "import rclpy; print('Success')"` | A quick "sanity check" to see if Python can find ROS. |
| `ros2 run <pkg> <script>.py` | Directly runs a Python node if it has a shebang (`#!/usr/bin/env python3`). |
| `autopep8 --in-place <file>.py` | Automatically formats your ROS 2 Python code to style standards. |

### 📂 Box 58: Workspace Information

| Command | Function |
| --- | --- |
| `colcon list --names-only` | A clean list of every package name in your workspace. |
| `colcon info <pkg_name>` | Shows the path and build type of a specific package. |
| `colcon build --build-base my_build` | Changes the name of the `build` folder to something else. |
| `colcon build --install-base my_install` | Changes the name of the `install` folder. |

### 📜 Box 59: Logging Levels & Filtering

| Command | Function |
| --- | --- |
| `ros2 run <pkg> <exe> --ros-args --log-level info` | Default logging level. |
| `ros2 run <pkg> <exe> --ros-args --log-level warn` | Only shows warnings and errors. |
| `ros2 run <pkg> <exe> --ros-args --log-level error` | Hides everything except critical system failures. |
| `ros2 run <pkg> <exe> --ros-args --log-level fatal` | Only shows errors that cause the node to crash. |

### 🏁 Box 60: System Cleanup & Reset

| Command | Function |
| --- | --- |
| `killall -9 ruby` | Stops Gazebo Classic if it hangs in the background. |
| `killall -9 gzserver` | Stops the Ignition/Gazebo Sim background process. |
| `ros2 daemon stop` | Kills the background ROS 2 discovery process to refresh the topic list. |
| `rm -rf ~/.ros/log/*` | Deletes all old log files to free up disk space. |


### 📦 Box 61: Navigation2 (Nav2) Core Execution

| Command | Function |
| --- | --- |
| `ros2 launch nav2_bringup bringup_launch.py` | The main command to start Nav2 (planner, controller, etc.). |
| `ros2 launch nav2_bringup localization_launch.py` | Starts only the AMCL localization and Map Server. |
| `ros2 launch nav2_bringup navigation_launch.py` | Starts only the planning and control stack. |
| `ros2 run nav2_util lifecycle_status` | Checks if all Nav2 servers are currently "Active". |

### 🗺️ Box 62: SLAM Toolbox (Mapping)

| Command | Function |
| --- | --- |
| `ros2 launch slam_toolbox online_async_launch.py` | Starts real-time mapping for a moving robot. |
| `ros2 launch slam_toolbox online_sync_launch.py` | Starts mapping where time is synced (best for slow high-res scans). |
| `ros2 run slam_toolbox sync_slam_toolbox_node` | Manually runs the synchronous SLAM node. |
| `ros2 service call /slam_toolbox/save_map ...` | Triggers a service to save the current SLAM map to a file. |

### ⚙️ Box 63: Advanced Parameter Operations

| Command | Function |
| --- | --- |
| `ros2 param dump /node_name` | Displays all parameters of a node in YAML format on screen. |
| `ros2 param load /node_name params.yaml` | Dynamically loads a YAML file into a running node. |
| `ros2 param get /node_name use_sim_time` | Checks if a node is currently using simulation time or real time. |
| `ros2 run <pkg> <exe> --ros-args -p name:=value` | Starts a node and sets a specific parameter immediately. |

### 🛠️ Box 64: Specialized `tf2` Utilities

| Command | Function |
| --- | --- |
| `ros2 run tf2_ros tf2_echo map odom` | Shows the transform () between the map and the odometry frame. |
| `ros2 run tf2_ros tf2_echo odom base_link` | Shows the robot's current position relative to its start point. |
| `ros2 run tf2_ros static_transform_publisher ...` | Publishes a fixed link (e.g., base_link to laser_link). |
| `ros2 run tf2_tools view_frames` | Generates a `frames.pdf` visual map of your robot's joints. |

### 📡 Box 65: Sensor Data Inspection (LiDAR/Camera)

| Command | Function |
| --- | --- |
| `ros2 topic type /scan` | Verifies if your LiDAR is using `sensor_msgs/msg/LaserScan`. |
| `ros2 topic echo /scan --no-arr` | Echoes LiDAR data but hides the long array of numbers for readability. |
| `ros2 run image_transport list_transports` | Shows available camera compression methods (e.g., JPEG, PNG). |
| `ros2 run rviz2 rviz2 -d src/my_pkg/rviz/view.rviz` | Launches RViz with your specific sensor preset. |

### 🏗️ Box 66: C++ (ament_cmake) Build Specifics

| Command | Function |
| --- | --- |
| `colcon build --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON` | Shows every single command CMake runs (best for debugging). |
| `colcon build --cmake-target <target_name>` | Builds only a specific library or executable within a package. |
| `colcon build --cmake-args -DENABLE_TESTING=OFF` | Disables testing to speed up build time significantly. |
| `colcon build --cmake-clean-first` | Deletes the old build for a package before starting a new one. |

### 💾 Box 67: ROS 2 Bag Metadata & SQL

| Command | Function |
| --- | --- |
| `ros2 bag info <bag_file>` | Shows the size, duration, and topic count of a recording. |
| `sqlite3 <bag_file>.db ".tables"` | Accesses the raw database (for advanced data recovery). |
| `ros2 bag play <bag> --clock` | Publishes the simulation clock stored in the bag file. |
| `ros2 bag record -s mcap /topic` | Records in the newer, faster MCAP format instead of SQLite3. |

### 🔌 Box 68: Hardware Port Management

| Command | Function |
| --- | --- |
| `ls -l /dev/ttyUSB*` | Shows all connected USB-to-Serial devices (LiDAR/Arduino). |
| `udevadm monitor` | Shows real-time kernel events when you plug or unplug sensors. |
| `sudo chmod 777 /dev/ttyUSB0` | (Temporary Fix) Gives full permission to access a serial port. |
| `dmesg | tail -n 20` |

### ⏱️ Box 69: ROS 2 Lifecycle Management

| Command | Function |
| --- | --- |
| `ros2 lifecycle list /node_name` | Lists all available transitions for a managed node. |
| `ros2 lifecycle set /node_name shutdown` | Cleanly shuts down a lifecycle node. |
| `ros2 lifecycle get /node_name` | Returns the current state (e.g., `active` or `unconfigured`). |
| `ros2 node info --show-lifecycle /node_name` | Shows node info including its lifecycle state. |

### 🐧 Box 70: Essential Bash Shortcuts for ROS 2

| Command | Function |
| --- | --- |
| `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash` | Enables "Tab Completion" for all `colcon` commands. |
| `export ROS_DOMAIN_ID=$((RANDOM%232))` | Assigns a random domain ID (useful for classroom environments). |
| `history | grep "colcon build"` |
| `alias r-clean='rm -rf build/ install/ log/'` | Custom alias to wipe a workspace instantly. |


### 📦 Box 71: ROS 2 Actions (The "Goal-Feedback-Result" Pattern)

Actions are used for long-running tasks like "Navigate to Point A."

| Command | Function |
| --- | --- |
| `ros2 action list -t` | Lists all actions and their message types. |
| `ros2 action info /navigate_to_pose` | Shows which nodes are the "Action Client" and "Action Server." |
| `ros2 action send_goal /action_name <type> "{data}"` | Manually sends a goal to a robot. |
| `ros2 action send_goal /action_name <type> "{data}" --feedback` | Sends a goal and prints live progress updates. |

### 🛠️ Box 72: Custom Interface Development (ament_cmake)

Commands used when you are writing your own `.msg` or `.srv` files.

| Command | Function |
| --- | --- |
| `rosidl_generate_interfaces` | (CMake macro) The core command inside `CMakeLists.txt` to build messages. |
| `ros2 run rosidl_runtime_py msg_to_yaml <pkg> <msg>` | Converts a message structure to a YAML template. |
| `colcon build --packages-select <msg_pkg>` | Build only your message package (must be done before nodes use them). |
| `ros2 interface show <pkg>/msg/<MsgName>` | Verifies your custom message was compiled correctly. |

### 👁️ Box 73: RViz2 Advanced Interaction

| Command | Function |
| --- | --- |
| `rviz2 --help-all` | Shows advanced graphics flags (e.g., forcing a specific OpenGL version). |
| `ros2 run rviz2 rviz2 -s <splash_image>` | Customizes the RViz startup screen (cool for branding your robot). |
| `ros2 run rviz2 rviz2 --fixed-frame odom` | Forces RViz to start using the `odom` frame as the anchor. |
| `ros2 run rviz2 rviz2 -d <config>.rviz` | Loads a saved environment with LiDAR, Camera, and Map displays ready. |

### 🌍 Box 74: Gazebo "Ignition" (New Gazebo) Basics

| Command | Function |
| --- | --- |
| `ign gazebo -v 4` | Starts Gazebo with high verbosity (shows detailed error logs). |
| `ign gazebo -s` | Starts the Gazebo server only (no GUI, saves CPU). |
| `ign gazebo -g` | Starts the Gazebo GUI only (to connect to a running server). |
| `ign gazebo -r <world_file>` | Starts the simulation and plays physics immediately. |

### 🔐 Box 75: SROS2 Access Control (Security)

| Command | Function |
| --- | --- |
| `ros2 security create_permission <keystore> <enclave> <policy.xml>` | Generates the rules for what a node can and cannot do. |
| `ros2 security list_keys <keystore>` | Shows all generated certificates for your robots. |
| `ros2 security check_policy <policy.xml>` | Validates your security XML for syntax errors. |
| `export ROS_SECURITY_LOOKUP_STRATEGY=Match` | Ensures nodes only talk if their security names match exactly. |

### 📡 Box 76: Discovery Server (High-Performance Networking)

| Command | Function |
| --- | --- |
| `fastdds discovery --server-id 0` | Starts a dedicated discovery server (replaces simple multicast). |
| `export ROS_DISCOVERY_SERVER="127.0.0.1:11811"` | Points a node to the discovery server for communication. |
| `ros2 run fastdds_statistics_tool visualizer` | Opens a web-based dashboard to monitor DDS traffic. |
| `ros2 daemon stop` | Clears the local discovery cache (fixes "ghost" topics). |

### 🤖 Box 77: Automated Code Quality (Linters)

| Command | Function |
| --- | --- |
| `ament_copyright .` | Checks every file in your package for a proper license header. |
| `ament_cppcheck <file>` | Scans C++ code for logic errors (memory leaks, null pointers). |
| `ament_uncrustify --reformat <file>` | Automatically fixes the indentation of your C++ code. |
| `ament_pep257 <file>` | Checks Python docstrings for proper documentation style. |

### 📂 Box 78: VCS (Version Control) Advanced Usage

| Command | Function |
| --- | --- |
| `vcs diff src` | Shows the `git diff` for every repository in your workspace at once. |
| `vcs log --limit 5 src` | Shows the last 5 commits for every package in your `src` folder. |
| `vcs validate src < my_robot.repos` | Checks if your `.repos` file matches what is actually in `src`. |
| `vcs custom --args status` | Runs a custom command across all repositories in the folder. |

### 📈 Box 79: System Monitoring (Detailed Analysis)

| Command | Function |
| --- | --- |
| `iotop` | Shows which ROS process is writing the most data to the disk (useful for Bags). |
| `nload` | Shows real-time network bandwidth usage (checks if LiDAR is flooding Wi-Fi). |
| `glxinfo | grep "OpenGL"` |
| `cat /proc/cpuinfo | grep "MHz"` |

### ⏱️ Box 80: Clock & Time Synchronization

| Command | Function |
| --- | --- |
| `ros2 topic delay /topic_name` | Measures the time difference between message creation and arrival. |
| `ros2 param get /node_name use_sim_time` | Checks if the node is following the system clock or Gazebo clock. |
| `timedatectl status` | Checks if your Linux system clock is synchronized with the internet (NTP). |
| `ros2 topic echo /clock` | Views the master clock being published by the simulator. |

### 📦 Box 81: Nav2 Costmap Manipulation

Costmaps are the "layers" that tell a robot where it can and cannot go.

| Command | Function |
| --- | --- |
| `ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.5` | Changes how far the robot stays away from walls live. |
| `ros2 topic echo /global_costmap/costmap` | Views the raw occupancy grid data of the global map. |
| `ros2 run nav2_map_server map_saver_cli -f my_map --ros-args -p save_map_timeout:=10000` | Increases timeout for saving very large high-res maps. |
| `ros2 lifecycle set /controller_server activate` | Manually restarts the path-following controller. |

### 🏗️ Box 82: Industrial ROS (Universal Robots, ABB, Fanuc)

Essential for working with large robotic arms and factory automation.

| Command | Function |
| --- | --- |
| `ros2 launch ur_bringup ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.100` | Connects ROS 2 to a physical Universal Robot. |
| `ros2 launch moveit_setup_assistant setup_assistant.launch.py` | Opens the GUI to configure motion planning for a robotic arm. |
| `ros2 control list_hardware_interfaces` | Checks if the physical robot motor controllers are connected. |
| `ros2 control list_controllers` | Shows if the "Joint Trajectory Controller" is active or inactive. |

### 🛰️ Box 83: Multi-Robot Frames & Namespacing

When running two robots (e.g., `robot1` and `robot2`) simultaneously.

| Command | Function |
| --- | --- |
| `ros2 run robot_state_publisher robot_state_publisher --ros-args -r __ns:=/robot1` | Runs a node inside a specific namespace. |
| `ros2 run tf2_ros static_transform_publisher ... --frame-id world --child-frame-id robot1/odom` | Links multiple robots to a single "World" frame. |
| `ros2 topic list | grep robot1` |
| `export ROS_DOMAIN_ID=1` | Isolates communication so Robot 1 and Robot 2 don't "hear" each other. |

### 🛠️ Box 84: Advanced MoveIt 2 (Motion Planning)

| Command | Function |
| --- | --- |
| `ros2 launch moveit_kinematics_test.launch.py` | Runs a stress test on the Inverse Kinematics (IK) solver. |
| `ros2 topic echo /display_planned_path` | Views the "ghost" trajectory in RViz before the arm moves. |
| `ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{...}"` | Manually calculates joint angles for a 3D coordinate. |
| `ros2 param set /move_group allowed_execution_duration_scaling 2.0` | Slows down the robotic arm for safety during testing. |

### 🧪 Box 85: Simulation Hardware Acceleration (GPU)

| Command | Function |
| --- | --- |
| `export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/path/to/plugin` | Adds custom C++ physics plugins to Gazebo. |
| `nvidia-smi` | Checks if your NVIDIA GPU is actually being used by the simulation. |
| `export LIBGL_ALWAYS_SOFTWARE=1` | Forces CPU rendering (use this if RViz crashes on your graphics card). |
| `ign gazebo -v 4 --render-engine ogre2` | Uses the advanced Ogre2 engine for better lighting in Gazebo. |

### 📜 Box 86: ROS 2 Launch Introspection

| Command | Function |
| --- | --- |
| `ros2 launch --print-description <pkg> <file>` | Prints the internal logic of a launch file without running it. |
| `ros2 launch <pkg> <file> --show-args` | Lists every variable you can toggle (e.g., `use_sim_time`, `headless`). |
| `ros2 launch <pkg> <file> -d` | Launch with "Debug" mode enabled to find syntax errors in Python. |

### 🔌 Box 87: CAN Bus & Serial (Industrial Sensors)

| Command | Function |
| --- | --- |
| `sudo ip link set can0 up type can bitrate 500000` | Initializes a CAN port for motor drivers or high-end sensors. |
| `candump can0` | Monitors raw hex data coming from the robot's hardware bus. |
| `cansend can0 123#DEADBEEF` | Sends a raw hex command to a specific motor ID. |
| `screen /dev/ttyUSB0 115200` | Opens a direct terminal to a microcontroller (press `Ctrl+A`, then `K` to exit). |

### ⏱️ Box 88: Real-Time Linux Tuning (RT-Preempt)

Commands for robots that require microsecond precision.

| Command | Function |
| --- | --- |
| `uname -a | grep PREEMPT_RT` |
| `chrt -p 99 <PID>` | Sets a specific ROS node to the highest possible real-time priority. |
| `cyclictest -t1 -p 80 -n -i 10000` | Measures the "latency jitter" of your computer's CPU. |

### 📦 Box 89: Custom RMW (Middleware) Switching

| Command | Function |
| --- | --- |
| `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` | Swaps to CycloneDDS (better for large data/images). |
| `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp` | Swaps to FastDDS (Standard default). |
| `ros2 run rmw_implementation_cmake show_rmw_implementation` | Confirms exactly which middleware is currently active. |

### 📂 Box 90: Workspace Clean & Deep Fix

| Command | Function |
| --- | --- |
| `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON` | Generates a JSON file for VS Code to "understand" your code. |
| `rm -rf build/ install/ log/` | The "Golden Rule" for fixing weird C++ compilation errors. |
| `rosdep update && rosdep install --from-paths src --ignore-src -y` | Automatically finds and installs every library your project needs. |


### 📦 Box 91: Docker for Robotics Production

Commands to containerize your robot for consistent deployment.

| Command | Function |
| --- | --- |
| `docker build --network=host -t my_robot .` | Builds an image using host networking (crucial for DDS). |
| `docker run --device /dev/video0:/dev/video0` | Passes a physical USB camera into a Docker container. |
| `docker run --gpus all -it my_ai_robot` | Allows a Dockerized ROS 2 node to access the host's NVIDIA GPU. |
| `docker system prune -a` | Deletes all unused images and containers to save disk space on the robot. |

### 🧠 Box 92: AI & Computer Vision (OpenCV/TensorFlow)

Integrating machine learning models into the ROS 2 pipeline.

| Command | Function |
| --- | --- |
| `ros2 run image_tools showimage --ros-args -r image:=/camera_out` | Visualizes the output of an AI processing node. |
| `ros2 topic echo /bounding_boxes` | Monitors coordinates sent by an object detection node (like YOLO). |
| `ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:=[640,480]` | Sets resolution for AI model input. |
| `ros2 run vision_msgs_visualizer visualizer_node` | Renders AI detection boxes directly onto the image stream. |

### 🌐 Box 93: ROS 2 Web & Cloud Bridge

For connecting your robot to a web dashboard or remote server.

| Command | Function |
| --- | --- |
| `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` | Starts a WebSocket to talk to a web browser (JavaScript/React). |
| `ros2 run web_video_server web_video_server` | Streams robot camera feeds to a web URL (HTTP). |
| `ros2 topic pub /web_cmd_vel geometry_msgs/Twist "{...}"` | Tests remote control commands coming from a web interface. |
| `export ROS_DOMAIN_ID=0` | (Reminder) Ensure the web bridge is on the same domain as the robot. |

### ⚡ Box 94: Hardware Acceleration (NVIDIA Isaac ROS)

For high-performance processing on Jetson or RTX GPUs.

| Command | Function |
| --- | --- |
| `ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py` | Starts GPU-accelerated tag tracking (very fast). |
| `ros2 run isaac_ros_dnn_encoders dnn_encoder_node` | Pre-processes images for deep learning using hardware. |
| `ros2 run tf2_ros tf2_monitor` | Monitors transform latency when using high-speed GPU SLAM. |

### 🛠️ Box 95: Professional CI/CD & Testing (Industrial)

Commands for automated code checking before pushing to GitHub.

| Command | Function |
| --- | --- |
| `colcon build --mixin coverage-gcc` | Prepares code to measure how much of it is covered by tests. |
| `ros2 run ament_index_cpp list_packages` | Verifies that the package index is consistent. |
| `ros2 run launch_testing_examples test_launch_example.py` | Runs a test that checks if nodes start up correctly. |
| `pytest src/my_package/test/` | Runs standard Python logic tests for your ROS nodes. |

### 🧩 Box 96: Advanced Topic Remapping

Useful for making different packages work together without changing code.

| Command | Function |
| --- | --- |
| `ros2 run <pkg> <node> --ros-args -r /old:=/new` | Simple topic rename. |
| `ros2 run <pkg> <node> --ros-args -r __node:=my_new_name` | Changes the name of the node itself at runtime. |
| `ros2 run <pkg> <node> --ros-args -r __ns:=/robot_A` | Puts the node into a specific namespace. |
| `ros2 run <pkg> <node> --ros-args --remap /cmd_vel:=/turtle1/cmd_vel` | Direct remapping for specific motion controllers. |

### 🔧 Box 97: Systemd (Running ROS 2 on Startup)

How to make your robot start its "brain" as soon as it's turned on.

| Command | Function |
| --- | --- |
| `sudo systemctl start my_robot.service` | Manually starts your robot's background service. |
| `sudo systemctl enable my_robot.service` | Sets the robot to start ROS 2 automatically on every boot. |
| `journalctl -u my_robot.service -f` | Follows the live logs of the startup service. |
| `sudo systemctl stop my_robot.service` | Shuts down the robot's background processes. |

### 📊 Box 98: Advanced RQt Troubleshooting

| Command | Function |
| --- | --- |
| `rqt --force-discover` | Forces RQt to find all nodes (fixes missing nodes in the graph). |
| `rqt_publisher` | Opens a GUI to manually publish messages to any topic. |
| `rqt_service_caller` | Opens a GUI to find and trigger any service on the robot. |
| `rqt_reconfigure` | Opens a GUI to change parameters (YAML) dynamically. |

### 📡 Box 99: Discovery Server & Large Networks

| Command | Function |
| --- | --- |
| `fastdds discovery --server-id 0 --ip-address 127.0.0.1 --port 11811` | Starts the master discovery server. |
| `export ROS_DISCOVERY_SERVER="192.168.1.10:11811"` | Connects a remote robot to the central server. |
| `ros2 daemon stop; ros2 daemon start` | Restarts the discovery daemon to clear network "ghosts." |

### 🏁 Box 100: The "Master Setup" One-Liner

The ultimate command to prepare a fresh terminal for work.

| Command | Function |
| --- | --- |
| `source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=42` | **The Essential Combo:** Sources ROS, sources your code, and sets your ID. |

---
