# Description: Install the GUI dependencies for the VSSS-VICE project

GUI=ssl-gui

# Enter VICE parent directory
cd ../

# Check if the GUI directory exists, if not, clone the repository and enter the directory
# else, enter the directory and pull the latest changes
if test ! -d .$GUI; then
    git clone https://github.com/Ararabots-UFMS/ssl-gui.git
    cd $GUI/
else  
    cd $GUI/
    git pull
fi

# Install GUI dependencies
npm install