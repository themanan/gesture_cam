from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    gesture_detection = Node(
        package="gesture_cam",
        executable="gesture_detection",
        output = 'screen'
    )
    
    gesture_control = Node(
        package="gesture_cam",
        executable="gesture_control",
        output = 'screen'
    )
    
    img_cap = Node(
        package="gesture_cam",
        executable="img_subscriber",
        output = 'screen'
    )


    ld.add_action(gesture_detection)
    ld.add_action(gesture_control)
    ld.add_action(img_cap)
    return ld