# Description: Install the GUI dependencies for the VSSS-VICE project

# Enter VICE parent directory
cd ../

# Check if the GUI directory exists, if not, clone the repository and enter the directory
# else, enter the directory and pull the latest changes
if test ! -d ./vsss-gui; then
    git clone https://github.com/Ararabots-UFMS/vsss-gui.git
    cd vsss-gui/
else  
    cd vsss-gui/
    git pull
fi

# Install GUI dependencies
npm install