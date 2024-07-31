import launch
import launch_ros.actions
from pathlib import Path

def generate_launch_description():
    # Define the path to the rosbag file
    bag_file = Path(__file__).parent / 'test_husky_0.db3'
    
    return launch.LaunchDescription([
        
        # Launch the rosbag2 player
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', str(bag_file)],
            output='screen'
        ),
        
        # Launch your ROS node
        launch_ros.actions.Node(
            package='collision',
            executable='talker',
            name='talker'
        )
        
        
    ])
