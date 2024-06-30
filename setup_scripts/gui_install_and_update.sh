# Description: Install the GUI dependencies for the VSSS-VICE project

# Enter VICE parent directory
while [ ! -d ./ssl-VICE ]; do
    cd ..
    if [ $PWD = "/" ]; then
        echo "Couldn't find the ssl-VICE directory. Please run this script from the ssl-VICE directory."
        exit 1
    fi
done

# Check if the GUI directory exists, if not, clone the repository and enter the directory
# else, enter the directory and pull the latest changes
if test ! -d ./ssl-gui; then
    echo "Couldn't find the ssl-gui directory. Cloning the repository..."
    git clone https://github.com/Ararabots-UFMS/ssl-gui.git
    cd ssl-gui/
else
    echo "The directory ssl-gui already exists. Pulling the latest changes..."
    cd ssl-gui/
    git pull
fi

# Install GUI dependencies
npm install