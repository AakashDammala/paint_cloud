import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess, AppendEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('paint_cloud')
    
    # Ensure Gazebo can find the package by adding the share directory to the model path
    # pkg_share is .../share/paint_cloud. We need .../share to be in the path
    # so that package://paint_cloud/ resolves correctly if treated as a model path root
    install_share_dir = os.path.join(pkg_share, '..')
    
    set_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=install_share_dir
    )

    rviz_config_file = os.path.join(pkg_share, 'rviz', 'paint_cloud_classic.rviz')
    
    # Default paths from install directory
    default_urdf_path = os.path.join(pkg_share, 'urdf', 'my_ur5_robot.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds', 'paint.world')

    # --- Arguments ---
    # 1. Path to your saved URDF file
    urdf_file = LaunchConfiguration('urdf_file')
    declare_urdf = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='Path to the URDF file to spawn'
    )

    # 2. Path to the World file
    world_file = LaunchConfiguration('world_file')
    declare_world = DeclareLaunchArgument(
        'world_file',
        default_value=default_world_path,
        description='Path to the Gazebo world file'
    )

    # --- Nodes ---

    # 1. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={'world': world_file}.items(),
    )

    # 2. Robot State Publisher
    # We read the URDF file content to pass as a parameter
    # Note: We use xacro to allow $(find pkg) syntax resolution
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
        ]
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_content}]
    )

    # 3. Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ur_manipulator',
                   '-x', '0', '-y', '0', '-z', '0.1'], # Raise slightly so it doesn't clip ground
        output='screen'
    )

    # 4. Spawners for Controllers (Standard for UR5)
    # Since your URDF has ros2_control tags, we need to activate the broadcasters
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    traj_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Ensure controllers start ONLY after the robot is spawned
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster, traj_controller],
        )
    )

    lidar_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar_tf.launch.py')
        )
    )

    return LaunchDescription([
        set_gazebo_model_path,
        declare_urdf,
        declare_world,
        gazebo,
        rviz,
        robot_state_publisher,
        spawn_entity,
        delay_controllers,
        lidar_tf
    ])