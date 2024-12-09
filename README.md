# ROS 1 Project with VS Code DevContainer

Welcome to the ROS 1 (Melodic) Project! This repository is configured to work seamlessly with VS Code using a DevContainer. It includes the necessary setup to quickly build, source, and run the project with minimal effort.

# Prerequisites

1.	Docker: Ensure you have Docker installed and running on your machine.
2.	Visual Studio Code:

    * Install VS Code.

    * Install the Dev Containers extension (ms-vscode-remote.remote-containers).

3.	ROS 1 Workspace: This project assumes a ROS 1 workspace structure.

# Features

* Pre-configured DevContainer for ROS 1 development.

* Environment variables (e.g., ROS_MASTER_URI) are pre-set.

* Minimal setup required—just build, source, and run!

## Getting Started

1. Clone the Repository


2.	Open the folder in VS Code


3.	Reopen the project in the DevContainer environment. VS Code will prompt you to do this automatically if the DevContainer configuration is detected.

## 2. Build and Run the Project

Inside the DevContainer terminal, execute the following commands:

1.	Build the project:

```bash
catkin_make
```

2.	Source the workspace:

```bash
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```

3.	Launch the system controller:

```bash
roslaunch task_controller system_controller.launch
````

#Environment Configuration

* ROS_MASTER_URI: Pre-configured in the DevContainer. Ensure your ROS network configuration matches your requirements.

* Workspace Directory: The default workspace is located at /workspaces/<repository_name>.

# Project Structure

```
.
├── requirements.txt                        # Python 2.7 dependencies
└── src
    ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
    └── task_controller
        ├── CMakeLists.txt                  # CMake configuration file
        ├── common_modules
        │   ├── motion                      # Common motion control modules
        │   │   ├── motion_control.py       # Motion control module
        │   │   └── trajectory.py           # Trajectory generation module
        │   ├── object_detection            # Common object detection modules
        │   │   ├── aruco_detection.py      # ArUco detection module
        │   │   └── color_detection.py      # Color detection module
        │   ├── positioning                 # Common positioning modules
        │   │   ├── beacon_utils.py         # Beacon utility functions
        │   │   └── localization.py         # Localization module
        │   ├── sensors                     # Common sensor processing modules
        │   │   ├── camera_processing.py    # Camera processing module
        │   │   └── lidar_processing.py     # LiDAR processing module
        │   └── utils                       # Common utility modules
        │       ├── conversions.py          # Unit conversion functions
        │       └── geometry_utils.py       # Geometry utility functions
        ├── launch                          # ROS launch files
        │   └── system_controller.launch    # System controller launch file
        ├── package.xml                     # ROS package configuration
        ├── scripts                         # ROS scripts
        │   └── node_controller.py          # Node controller script
        └── tasks                           # ROS task modules
            ├── task_beacon_positioning.py  # Task 4: beacon positioning module
            ├── task_color_sorting.py       # Task 6: color sorting module
            ├── task_line_following.py      # Task 2: line following module
            ├── task_navigation.py          # Task 3: navigation module
            ├── task_object_handling.py     # Task 5: object handling module
            └── task_trajectory.py          # Task 1: trajectory module (Hecho, falta calibrar el giro)
````

# Debugging and Development

Use VS Code’s built-in tools for:
*	ROS debugging: Leverage breakpoints, variable watches, and integrated terminals.

*	Linting: Check ROS code quality with supported extensions.

# Troubleshooting

If you encounter any issues:
* Verify that Docker is running.
* Ensure the DevContainer builds successfully.
* Confirm the ROS environment variables are correctly set by running:

```
printenv | grep ROS
```

* If issues persist, rebuild the workspace:

```
catkin_make clean
catkin_make
````

As a note, you need to change the ROS_IP to your own IP address in .bashrc file.

# Contributing

Contributions are welcome! Feel free to submit issues or pull requests to improve the project.

# License

This project is licensed under the MIT License.

Enjoy developing with ROS 1 in a seamless DevContainer environment! 🚀