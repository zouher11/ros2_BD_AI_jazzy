import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    pkg_share = get_package_share_directory('slam_mapping_tp')

    world = os.path.join(
        pkg_share, 'worlds', 'lab_world.wbt'
    )

    # Standalone Python controller supplied with the project
    driver_script = os.path.join(
        pkg_share,
        'controllers', 'go2_driver', 'go2_driver.py'
    )

    webots_home = os.environ.get(
        'WEBOTS_HOME', '/usr/local/webots'
    )

    # Start Webots with the simulation enabled
    webots = WebotsLauncher(
        world=world,
        gui=True,
        mode='realtime',
        ros2_supervisor=False,
        port='1234',
    )

    # Connect the existing standalone Python controller
    controller = ExecuteProcess(
        cmd=[
            os.path.join(webots_home, 'webots-controller'),
            '--protocol=ipc',
            '--port=1234',
            '--robot-name=robot',
            driver_script
        ],
        output='screen',
        additional_env={
            'WEBOTS_HOME': webots_home,
            'PYTHONUNBUFFERED': '1',
        }
    )

    # Allow Webots to initialize before connecting
    delayed_controller = TimerAction(
        period=10.0,
        actions=[controller]
    )

    base_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'base_link', 'base_footprint'
        ],
        output='screen'
    )

    return LaunchDescription([
        webots,
        delayed_controller,
        base_transform,
    ])
