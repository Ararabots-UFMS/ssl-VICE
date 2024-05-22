import os
import subprocess

####################
# This file was created taking into account that it will be run with
#   the following command: ros2 run gui run_gui
# If it is run any other way it probably won't work
# (Needs to be built with colcon and sourced beforehand)
####################

gui_repo_path = os.pardir+ "/vsss-gui"

def main():
    if not os.path.exists(gui_repo_path):
        subprocess.run(["bash", "setup_scripts/gui_install.sh"])
    
    os.chdir(gui_repo_path)
    subprocess.run(["npm", "run", "start"])


if __name__ == '__main__':
    main()