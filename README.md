Gesture-Controlled Wheeled Robot

This project implements a gesture-controlled wheeled robot using ROS2, OpenCV, and MediaPipe. The robot responds to hand gestures to control its movements, such as 'go', 'reverse', 'right', 'left', and 'stop'.

Project Overview

We use OpenCV and MediaPipe for real-time gesture recognition. The recognized gestures are published to specific ROS2 topics, which are then used to control the movement of the wheeled robot.

Recognized Gestures
- **Go:** Move forward
- **Reverse:** Move backward
- **Right:** Turn right
- **Left:** Turn left
- **Stop:** Halt movement

Files and Usage

gesture_detection.py
This script detects hand gestures and publishes the recognized gesture on the 'gesture_recog' topic. It also publishes the camera feed to the 'video_publisher' topic.

#### Run Command:
```bash
ros2 run gesture_cam gesture_detection
```

### `gesture_control.py`
This script listens to the `gesture_recog` topic for recognized gestures and controls the robot’s movements by publishing velocity commands to the `/cmd_vel` topic.

#### Run Command:
```bash
ros2 run gesture_cam gesture_control
```

### `img_subscriber.py`
This script opens a window to visualize the camera feed and the detected gestures.

#### Run Command:
```bash
ros2 run gesture_cam img_subscriber
```

Launch All Scripts
You can launch all the above scripts simultaneously using the provided launch file.

Launch Command:
```bash
ros2 launch gesture_cam gesture_control.launch.py
```

Launch the Robot in Gazebo
To launch the robot in a Gazebo simulation environment, use the following command:

```bash
ros2 launch pickup_bot pickup_bot_empty_world.launch.xml
```

## Requirements

- ROS2 Humble
- OpenCV
- MediaPipe
- Python 3.x

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/username/gesture-controlled-robot.git
   ```
2. Clone this repository:
```bash
   git clone https://github.com/username/pickup-bot.git
3. Build the ROS2 package:
   ```bash
   colcon build
   ```

Feel free to modify this to better fit your project's specifics!
