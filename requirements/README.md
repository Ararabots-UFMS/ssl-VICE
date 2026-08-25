# Installing Requirements

### Python
First of all, let's install python and ROS2 Humble before setup ssl-VICE.

To install python, use the following commands:

```bash
sudo apt update # update system packages
sudo apt install python3 # install python latest version
```

To check that the installation was successful, use the following command:
```bash
python3 --version # check python version
```

After that, if the command works python is succesfully installed.

### ROS 2 Humble Base Version
First of all, we need to make sure that we have support to UTF-8, for this use the following commands:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

After that, we need to add ROS2 repository in our system with the following commands:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```
Then we have to add ROS2 GPG key with the following command:
```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
And Then add the repository to your sources list with the following command:
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Now, we have to install the ROS2 Humble Base package, for this use the following commands:
```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-ros-base
```

Finally, we need to set up the ROS 2 environment by adding it to the terminal configuration file, using this command:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Then check if ROS2 was succesfully installed using the following:
```bash
ros2
```

If it works, all the Requirements was succesfully installed and you can go back to the main installation tutorial for ssl-VICE.
