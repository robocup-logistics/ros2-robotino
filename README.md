
# robotino-driver

This Git repository contains an extensive set of code files designed for the Robotino driver using ROS2 middleware.

Drive Motor Control: Enables the control of Robotino's drive motors utilizing omniwheel drive kinematics and implementing inverse and forward kinematic models.

Battery Management: Provides mechanisms for managing the Robotino's battery, ensuring optimal usage and monitoring.

Odometry: Estimating the current position and orientation of a robot by analyzing data from its sensors, typically wheel encoders.

Sensor Interfaces:

 - Gyroscope: Interfaces for reading data from the gyroscope sensor, providing information on orientation and rotation.
 - Camera (Realsense): Interfaces for integrating and utilizing the camera module (Realsense) with Robotino.
 - Analog Input and Output Array: Interfaces for handling analog input and output signals, facilitating interaction with external devices.
 - Digital Input and Output Array: Interfaces for managing digital input and output signals, useful for interacting with various sensors and actuators.
 - Bumper Sensors: Interfaces for reading data from bumper sensors, enabling collision detection and avoidance.
 - Laser Range Finder(hokuyo): Interface for the laser range finder sensor, used for detecting obstacles and mapping the environment.

Encouraging collaboration and contributions, the repository aims to serve as a reliable resource for researchers, developers, and enthusiasts in the fields of robotics and simulation. Regular updates ensure compatibility with evolving dependencies, making it a sustainable and valuable tool for the robotics community.

## Table of content
- Installation
    - Dependencies
    - Installation from source
- Building
- Launch robotino driver - Bringup
- Launch robotino driver - Standalone
- Contributing to this repo
- Research and References
- Bugs and Issues
- Way-forward

## Installation

### Installing from source:
ATTENTION: These commands assume that you have created a workspace called "robotino_driver_ws" in your home folder. If you used a different directory or name, please adjust the commands accordingly.

Install the below debian packages first(part of the repository):

    rec-rpc_1.6.1_amd64.deb
    robotino-api2_1.0.0_amd64.deb
    robotino-dev_1.0.3_amd64.deb

After installing ROS2 and creating the workspace, clone this repository in your workspace:

    cd ~/robotino_driver_ws/src
    git clone https://github.com/carologistics/robotino-driver.git

Install the binary dependencies by running the following command in the root of your workspace:

    cd ~/robotino_driver_ws
    rosdep init
    rosdep update

    sudo apt update

    rosdep install --from-paths src/robotino-driver --ignore-src -r -y --rosdistro humble


If all dependencies are already installed, you should see the message "All required rosdeps installed successfully."

## Building:
To build the workspace, run the following command in the root of your workspace:

    cd ~/robotino_driver_ws
    colcon build --symlink-install

If the build is successful, you should see the message "Finished [build] target(s) in [time]s."

After building the package, open a new terminal and navigate to your workspace. Then, source the overlay by running the following command:

    source /opt/ros/humble/setup.bash

Then, source the workspace by running the following command:

    cd ~/robotino_driver_ws
    source install/setup.bash

## Launch robotino driver

### Launching robotino driver - Bringup


    ros2 launch rto_node robotino_driverbringup.launch.py namespace:=robotinobase1


- namespace: It's a launch configuration used to spawn the corresponding robotinobase, its controllers, and node parameters

### Launching robotino driver - Standalone

    ros2 launch rto_node robotino_driver.launch.py namespace:=robotinobase1

- namespace: It's a launch configuration used to spawn the corresponding robotinobase, its controllers, and node parameters

## Contributing to this repo

To contribute to this package, you can either open an issue describing the desired subject or develop the feature yourself and submit a pull request to the main branch (in this case, robot_cluster).

## Research and References
- Omnidirectional robot kinematics and dynamics:
  Moreno, J.; Clotet, E.; Lupiañez, R.; Tresanchez, M.; Martínez, D.; Pallejà, T.; Casanovas, J.; Palacín, J. Design, Implementation and Validation of the Three-Wheel Holonomic Motion System of the Assistant Personal Robot (APR). Sensors 2016, 16, 1658. [Google Scholar]

  Jordi Palacín; Elena Rubies; Eduard Clotet; and David Martínez; Evaluation of the Path-Tracking Accuracy of a Three-Wheeled Omnidirectional Mobile Robot Designed as a personal Assistant https://doi.org/10.3390/s21217216

- [ROS2](https://docs.ros.org/en/foxy/index.html)

- [Navigation2](https://navigation.ros.org/)

- [rec::robotino::api2 C++ API documentation](https://doc.openrobotino.org/download/RobotinoAPI2/rec_robotino_api2/index.html)

## Bugs and Issues
Please report bugs and request features using the Issue Tracker
