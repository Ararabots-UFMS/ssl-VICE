Software Requirements
---------------------

- [Python3](https://www.python.org/downloads/)
- [ROS2](https://docs.ros.org/en/humble/index.html)
- [Node] (for linux users, it's advisable to use nvm to install node and nest)(https://github.com/nvm-sh/nvm)
- [Go]
- [Docker]

Setup
---------------------
To run the game controller in localhost properly, you must clone the ssl-game-controller repo, go to the path you cloned and run the following command:
```
go run cmd/ssl-game-controller/main.go
```

After that, you must build the dockers in ssl-game-controller to connect the autoreferee, to do this just run:
```
docker compose up
```

After cloning the VICE repository it is necessary to build it, enter in root of the repository folder and use the following command:
```
colcon build
```

Usage
---------------------
After the setup just use ```ros2 run PACKAGE_NAME EXECUTABLE_NAME``` replacing the package and executable names with the ones you want to run.

Questions?
---------------------
Visit the the ssl-game-controller repo, available in: https://github.com/RoboCup-SSL/ssl-game-controller 