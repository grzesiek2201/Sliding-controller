from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnExecutionComplete
import math


def generate_launch_description():
    turtle_pkg = "coop_transportation"
    urdf_path = os.path.join(get_package_share_directory(turtle_pkg), 'models', 'turtlebot3_burger', 'model.sdf')

    # Launch configuration variables specific to simulation
    x_pose1 = LaunchConfiguration('x_pose1', default='4.65')
    y_pose1 = LaunchConfiguration('y_pose1', default='5.0')
    yaw_pose1 = LaunchConfiguration('yaw_pose1', default=str(math.pi/2))

    x_pose2 = LaunchConfiguration('x_pose2', default='5.35')
    y_pose2 = LaunchConfiguration('y_pose2', default='5.0')
    yaw_pose2 = LaunchConfiguration('yaw_pose2', default=str(math.pi/2))

    x_pose = [x_pose1, x_pose2]
    y_pose = [y_pose1, y_pose2]
    yaw_pose = [yaw_pose1, yaw_pose2]

    n_robots = 2

    robot_instances_cmds = []

    for i in range(n_robots):
        robot_cmd = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-entity', f'robot{i}',
                            '-file', urdf_path,
                            '-x', x_pose[i],
                            '-y', y_pose[i],
                            '-z', '0.01',
                            '-Y', yaw_pose[i],
                            '-robot_namespace', f'robot{i}'],
                output='screen'
            )
        robot_instances_cmds.append(robot_cmd)

    # box_file = os.path.join(get_package_share_directory('gazebo_ros'), 'models', 'box', 'model.sdf')
    # box_file = os.path.join(get_package_share_directory('coop_cpp'), 'models', 'box', 'model.sdf')
    # box = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'box',
    #                '-file', box_file,
    #                '-x', '5.0',
    #                '-y', '5',
    #                '-z', '0.2'],
    #             output='screen'
    # )

    ld = LaunchDescription()

    # ld.add_action(RegisterEventHandler(
    #     event_handler=OnExecutionComplete(
    #         target_action=robot_instances_cmds[-1],
    #         on_completion=[box],
    #     )
    # ))

    for cmd in robot_instances_cmds:
        ld.add_action(cmd)

    return ld

