import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
#regarder dans /opt/ros/rolling/lib/python3.8/site-packages/webots_ros2_driver/webots_launcher.py
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('my_package')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_robot.urdf')).read_text()
    #object_description = pathlib.Path(os.path.join(package_dir, 'resource', 'my_object.urdf')).read_text()
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()
    #Ros2Supervisor

    my_robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_robot'},
        parameters=[
            {'robot_description': robot_description,'set_robot_state_publisher': True},
              
        ]
    )

    """
    my_robot_driver2 = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Ros2Supervisor'},
        parameters=[
            {'robot_description': object_description},
              
        ]
    )
    """

    #publish poses in 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            #'robot_description': '<robot name="my_robot"><link name="translation"/></robot>'
            #'robot_description': '<robot name="my_robot"><link name="translation"/><link name="rotation"/></robot>'
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )


    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    
    """my_object_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_object'},
        parameters=[
            {'robot_description': object_description},
        ]
    )
    """

    return LaunchDescription([
        webots,
        my_robot_driver,
        robot_state_publisher,
        footprint_publisher,
        #my_object_driver,
        ros2_supervisor,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])