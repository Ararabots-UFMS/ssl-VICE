import subprocess
import pathlib
import os

####################
# This file was created taking into account that it will be run with
#   the following command: ros2 run gui run_gui
# If it is run any other way it probably won't work
# (Needs to be built with colcon and sourced beforehand)
####################

# Set folder names for both repos
VICE_REPO_NAME="ssl-VICE"
GUI_REPO_NAME="ssl-gui"

# Get the path to the file being executed
path_to_repo_parent_folder = pathlib.Path(__file__).parent.absolute()

# Find the parent folder to the VICE repo
while not (path_to_repo_parent_folder / VICE_REPO_NAME).exists():
    path_to_repo_parent_folder = path_to_repo_parent_folder.parent

# Add GUI repo name to the path
path_to_gui_repo = path_to_repo_parent_folder / GUI_REPO_NAME

def main():
    # If GUI path does not exist, run the installation script
    if not (path_to_gui_repo).exists():
        subprocess.run(["bash", "./setup_scripts/gui_install_and_update.sh"])

    # Change directory to GUI repo
    os.chdir(path_to_gui_repo)

    # Run the GUI
    subprocess.run(["npm", "run", "dev"])


if __name__ == '__main__':
    main()