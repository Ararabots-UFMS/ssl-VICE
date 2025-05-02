<div align="center">
<a href="https://quackfy.vercel.app/">
<img height="100" src="https://ararabots-ufms.github.io/img/arara_no_bg.png" alt="Arara">
</div>

<div align="center">
<img src="https://img.shields.io/badge/build-latest-blue">
<img src="https://img.shields.io/github/issues/Ararabots-UFMS/ssl-VICE">

</div>

## Software Requirements
- [Python3](https://www.python.org/downloads/)
- [ROS2](https://docs.ros.org/en/humble/index.html)


## Python Libraries
- Flask



## Installing Requirements

To learn how to install the requirement programs and packages, check the [Requirements README](./requirements/README.MD)

## Setup
---------------------
First of all, we need to clone the [ssl-VICE](https://github.com/Ararabots-UFMS/ssl-VICE) repository, using the following commmands:

```bash
git clone git@github.com:Ararabots-UFMS/ssl-VICE.git
```

After cloning the repository, enter the repository folder and build the repository with the following commands:
```bash
cd ssl-VICE/
colcon build # builds the repository
```
Now we have to setup the ssl-VICE environment by adding it to the terminal configuration file.

For this, let's get our setup.bash path, using the following commands:
```bash
cd install/
pwd # copy the path that is show in the terminal and add /setup.bash
```

pwd shows the current path that we are in the terminal, the final path to the setup.bash should be something like this:

```bash
.../ssl-VICE/install/setup.bash
```

Now to add the copied path to the terminal configuration file, use the following command:
```bash
echo 'source path' >> ~/.bashrc
```

Then, we have to add the path to the cloned repository to our terminal configuration file using the following commands:
```bash
cd ..
pwd #check if the final of the path is /ssl-VICE, then copy this path
echo 'export ARARA_VICE_PATH=path/to/repo' >> ~/.bashrc
```

Attention: make sure you have copied correctly the path to the repository folder and didn't leave a space after the equal sign

Finally, we have to install the requirement packages of **requirements.txt** file, for this use the following commands inside ssl-VICE folder:

```bash
pip install -r requirements.txt
```

If you get and error after that, probably it's about flask python package. To solve this, use the following commands:
```bash
pip install --upgrade pip setuptools wheel packaging scikit-build
pip install protobuf==3.20.*
```

After that, run again the following command and the ssl-VICE should be succesfully installed.
```bash
pip install -r requirements.txt
```

## Usage
After setup, let's check if everything works fine testing the gui_interpreter api Node, for this use the following command:
```bash
ros2 run gui_interpreter apiNode
```
If it works, ssl-VICE was succesfully installed.