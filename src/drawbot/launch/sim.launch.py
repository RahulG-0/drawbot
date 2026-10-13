"""Start Gazebo with the drawbot, the ROS bridge, and the drawbot nodes.

  ros2 launch drawbot sim.launch.py                 # Gazebo GUI
  ros2 launch drawbot sim.launch.py rviz:=true      # plus RViz
  ros2 launch drawbot sim.launch.py gui:=false      # headless server only
  ros2 launch drawbot sim.launch.py microsteps:=32  # finer stepper resolution
  ros2 launch drawbot sim.launch.py gpu:=false      # leave the GL driver alone
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('drawbot')
    world = os.path.join(share, 'worlds', 'drawing.sdf')
    xacro_file = os.path.join(share, 'urdf', 'drawbot.urdf.xacro')
    sim_time = {'use_sim_time': True}

    gui = LaunchConfiguration('gui')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(
                ["'-r ' + ('' if '", gui, "'.lower() == 'true' else '-s ') + '", world, "'"]),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # Under WSL2, Mesa falls back to software rendering (llvmpipe) unless told to
    # use the D3D12 driver, which is what reaches the real GPU through /dev/dxg.
    gpu_env = []
    if os.path.exists('/dev/dxg') and 'GALLIUM_DRIVER' not in os.environ:
        gpu_env.append(SetEnvironmentVariable(
            'GALLIUM_DRIVER', 'd3d12', condition=IfCondition(LaunchConfiguration('gpu'))))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('gpu', default_value='true',
                              description='Render on the GPU when running under WSL2'),
        DeclareLaunchArgument('microsteps', default_value='16',
                              description='Stepper microstepping (0 = ideal motors)'),
        *gpu_env,
        gazebo,
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': robot_description}]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-world', 'drawing', '-topic', 'robot_description', '-name', 'drawbot'],
             output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml')}],
             output='screen'),
        Node(package='drawbot', executable='controller', output='screen',
             parameters=[sim_time, {'microsteps': LaunchConfiguration('microsteps')}]),
        Node(package='drawbot', executable='trail', output='screen',
             parameters=[{'gazebo_markers': gui}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(share, 'rviz', 'drawbot.rviz')],
             parameters=[sim_time]),
    ])
