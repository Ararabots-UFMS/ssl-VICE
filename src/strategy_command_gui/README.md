# Strategy Command GUI

A ROS2 graphical interface for testing the PathDriver by sending `strategy_command` service requests.

## Features

- **Intuitive GUI**: Easy-to-use tkinter interface for sending commands
- **Service Status**: Real-time monitoring of service availability
- **Preset Commands**: Quick buttons for common positions (center, goal, corner)
- **Response Monitoring**: Display of service responses and status
- **Input Validation**: Error handling for invalid inputs

## Installation

1. Build the package:
```bash
cd /home/fabio/Repos/ssl-VICE
colcon build --packages-select strategy_command_gui
source install/setup.bash
```

## Usage

### Option 1: Run individual nodes

1. Start the PathDriver:
```bash
ros2 run new_movement driver
```

2. Run the Strategy Command GUI:
```bash
ros2 run strategy_command_gui strategy_gui
```

### Option 2: Use the launch file

```bash
ros2 launch strategy_command_gui test_path_driver.launch.py
```

## GUI Controls

- **Robot ID**: Select which robot to command (0-15)
- **Position X/Y**: Target position in millimeters
- **Velocity X/Y**: Target velocity in mm/s
- **Send Command**: Send the strategy command
- **Clear**: Reset all fields to zero
- **Preset Commands**: Quick buttons for common test positions

## Service Interface

The GUI sends requests to the `strategy_command` service with the following interface:

**Request:**
- `uint32 id` - Robot ID
- `float32 position_x` - Target X position (mm)
- `float32 position_y` - Target Y position (mm)
- `float32 velocity_x` - Target X velocity (mm/s)
- `float32 velocity_y` - Target Y velocity (mm/s)

**Response:**
- `bool sucess` - Whether the command was successful

## Testing with grSim

To test with grSim simulator:

1. Start grSim
2. Start your grSimPublisher node
3. Run the PathDriver and GUI as described above
4. Use the GUI to send commands and observe robot movement in grSim

## Notes

- The GUI automatically checks service availability every second
- Send button is disabled when service is unavailable
- Response history is maintained in the text area
- All coordinates are in millimeters as expected by the PathDriver
