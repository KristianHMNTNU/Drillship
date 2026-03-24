from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js2'   # <-- endre hvis din er js0/js1/js2
        }]
    )

    ps5_node = Node(
        package='drillship_sim',
        executable='ps5_node',      # <-- må matche setup.py entry_points
        name='ps5_node'
    )

    ta_node = Node(
        package='drillship_sim',
        executable='ta_node',         # <-- må matche setup.py entry_points
        name='ta_node'
    )

    return LaunchDescription([
        joy_node,
        ps5_node,
        ta_node
    ])