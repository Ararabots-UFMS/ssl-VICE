
Software Requirements
---------------------

- [Python3](https://www.python.org/downloads/)
- [ROS2](https://docs.ros.org/en/humble/index.html)

Python Libraries
---------------------

- Flask

Setup
---------------------

After cloning the repository it is necessary to build it, enter the repository folder and use the following command:
```
colcon build
```

Then add the enviroment variable with the path to the VICE folder to your .bashrc file:
```
echo 'export ARARA_VICE_PATH = path/to/repo' >> ~/.bashrc 
```

Usage
---------------------
After the setup just use ```ros2 run PACKAGE_NAME EXECUTABLE_NAME``` replacing the package and executable names with the ones you want to run.
