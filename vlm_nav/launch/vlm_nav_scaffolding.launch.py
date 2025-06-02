from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch the BasePrompter node (calls the base_prompt service once at startup)
        Node(
            package='vlm_nav',
            executable='base_prompter',
            name='base_prompter',
            output='screen'
        ),

        # Launch the ProblemManager node (subscribes to vlm_waypoints, publishes task_feedback)
        Node(
            package='vlm_nav',
            executable='problem_manager',
            name='problem_manager',
            output='screen'
        ),
    ])
