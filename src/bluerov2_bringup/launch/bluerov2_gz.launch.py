from os import environ
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('x', default_value=['0'],
        description='x position'),
    DeclareLaunchArgument('y', default_value=['0'],
        description='y position'),
    DeclareLaunchArgument('z', default_value=['-5'],
        description='z position'),
    DeclareLaunchArgument('roll', default_value=['0'],
        description='roll position'),
    DeclareLaunchArgument('pitch', default_value=['0'],
        description='pitch position'),
    DeclareLaunchArgument('yaw', default_value=['0'],
        description='yaw position'),    
    DeclareLaunchArgument('localization', default_value='off',
                          choices=['off', 'localization', 'slam'],
                          description='Whether to run localization or SLAM'),
    DeclareLaunchArgument('nav2', default_value='false',
                          choices=['true', 'false'],
                          description='Run nav2'),
    DeclareLaunchArgument('world', default_value='bluerov2_underwater',
                          description='Gazebo World'),
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Run bridges'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'],
                          description='Run description'),
    DeclareLaunchArgument('world', default_value='basic_map',
                          description='GZ World'),
    DeclareLaunchArgument('spawn_model', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn bluerov2 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('log_level', default_value='error',
                          choices=['info', 'warn', 'error'],
                          description='log level'),
]


def generate_launch_description():

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[('gz_args', [
            LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'
            ])]
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gz_ros_clock',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # Stereo camera image bridge
    stereo_camera_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='stereo_bridge_gz_ros_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['/model/bluerov2/stereo_camera/left/image_raw',
                   '/model/bluerov2/stereo_camera/right/image_raw',
        ])

    # Stereo camera image bridge
    stereo_camera_image_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='stereo_bridge_info_gz_ros_camera',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['/model/bluerov2/stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/model/bluerov2/stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ])

    joy = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            arguments=['dev=/dev/input/js1'
            ])
    bluerov2_teleop= Node(
            package='bluerov2_teleop',
            executable='joy_teleop',
            name='bluerov2_teleop',
            output='screen',
            arguments=[])
    
    orbslam3_mono= Node(
        package='ros2_orbslam3',
        executable='mono',
        output='screen',
        arguments=['/home/user/ORB_SLAM3/Vocabulary/ORBvoc.txt', '/home/user/colcon_ws/src/orbslam3_wrapper/gazeboCameraParameters.yaml','true'])


    #ros2 run ros2_orbslam3 mono /home/user/ORB_SLAM3/Vocabulary/ORBvoc.txt /home/user/colcon_ws/src/orbslam3_wrapper/gazeboCameraParameters.yaml true 




    # Define LaunchDescription variable
    return LaunchDescription(ARGUMENTS + [
        #robot_description,
        stereo_camera_image_bridge,
        stereo_camera_image_info_bridge,
        gz_sim,
        clock_bridge,
        joy,
        bluerov2_teleop,
        orbslam3_mono
        #nav2,
        #slam,
        #localization,
    ])
'''

    # Robot description
    #robot_description = IncludeLaunchDescription(
        #PythonLaunchDescriptionSource([PathJoinSubstitution(
        #[get_package_share_directory('b3rb_description'), 'launch', 'robot_description.launch.py'])]),
        #condition=IfCondition(LaunchConfiguration('description')),
        #launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'bluerov2_nav2'), 'launch', 'nav2.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('nav2')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])


    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'orbslam3'), 'launch', 'slam.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'slam'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('sync', LaunchConfiguration('sync'))])

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory(
        'bluerov2_localization'), 'launch', 'localization.launch.py'])]),
        condition=LaunchConfigurationEquals('localization', 'localization'),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('map', PathJoinSubstitution([get_package_share_directory(
                'bluerov2_nav2'), 'maps', LaunchConfiguration('map_yaml')]))])
    '''
