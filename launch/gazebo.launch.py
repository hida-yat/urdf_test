import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='urdf_test').find('urdf_test')
    default_model_path = os.path.join(pkg_share, 'urdf/test.urdf')
    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()
    world = os.path.join(
        get_package_share_directory('urdf_test'),
        'worlds',
        'nhk2025_field.world'
    )
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}] # add other parameters here if required #
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world, '-s': 'libgazebo_ros_init.so', '-s': 'libgazebo_ros_factory.so', 'use_sim_time': 'True'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    spawn_entity = launch_ros.actions.Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'urdf_test', '-topic', 'robot_description'],
    output='screen',
    parameters=[{'use_sim_time': True}]
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        #launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose',  '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        gzserver_cmd,
        gzclient_cmd,
        spawn_entity,
    ])