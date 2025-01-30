from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crazyfly_core',
            executable='optitrack_subscriber',
            name='optitrack_subscriber_node',
            output='screen'
        ),
        Node(
            package='crazyfly_core',
            executable='optitrack_subscriber2',
            name='optitrack_subscriber_node2',
            output='screen'
        ),
        Node(
            package='crazyfly_core',
            executable='cf_command_controller2',
            name='cf_command_controller_node2'
            #output='screen'
        )
        
    ])


#In case this doesnt work delete the last two Nodes 
# this includes cf_2 and optitrack_2 