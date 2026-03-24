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

    ta_node = Node(
        package='drillship_sim',
        executable='ta_node',         # <-- må matche setup.py entry_points
        name='ta_node'
    )

    npo_node = Node(
        package='drillship_sim',
        executable='npo_node',         # <-- må matche setup.py entry_points
        name='npo_node'
    )

    controller_node = Node(
        package='drillship_sim',
        executable='controller_node',         # <-- må matche setup.py entry_points
        name='controller_node'
    )

    pathplanner_node = Node(
        package='drillship_sim',
        executable='pathplanner_node',         # <-- må matche setup.py entry_points
        name='pathplanner_node'
    )

    return LaunchDescription([
        joy_node,
        ta_node,
        controller_node,
        npo_node,
        pathplanner_node
    ])