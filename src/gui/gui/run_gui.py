import subprocess
import pathlib
import os
import warnings

####################################################################
# This file was created taking into account that it will be run with
#   the following command: ros2 run gui run_gui
# If it is run any other way it probably won't work
# Needs to be built with colcon and sourced beforehand
####################################################################

# Set folder names for both repos
GUI_REPO_NAME="ssl-gui"

# Get the path to the VICE repository
path_to_repo_folder = pathlib.Path(os.environ['ARARA_VICE_PATH'])

# Get the path to the parent folder of the VICE repository
path_to_repo_parent_folder = path_to_repo_folder.parent

# Add GUI repository name to the path
path_to_gui_repo = path_to_repo_parent_folder / GUI_REPO_NAME

def main():
    # If GUI path does not exist, run the installation script
    if not (path_to_gui_repo).exists():
        warnings.warn(f"GUI repository not found at {path_to_gui_repo}. Running installation script")
        subprocess.run(["bash", f"{path_to_repo_folder}/setup_scripts/gui_install_and_update.sh"])

    # Change directory to GUI repo
    os.chdir(path_to_gui_repo)

    # Run the GUI
    gui_command = ["sudo", "npm", "run", "dev"]
    subprocess.run(gui_command)

    #TODO: Run gui_interpreter apiNode also

if __name__ == '__main__':
    main()