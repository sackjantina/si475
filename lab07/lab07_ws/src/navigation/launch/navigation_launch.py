from launch import LaunchDescription 
from ament_index_python.packages import get_package_share_directory 
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, GroupAction , DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution , LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace')
]


def generate_launch_description(): 
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup') 
    pkg_dir = get_package_share_directory('navigation') 
    turtlebot4_viz = get_package_share_directory('turtlebot4_viz') 

    localization_params_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [pkg_dir, 'config', 'localization.yaml']),
        description='Localization parameters')

    map_arg = DeclareLaunchArgument ( 
            'map', 
            default_value = PathJoinSubstitution (
                [pkg_dir, 'map', 'classroom_map_1.yaml']), 
            description="Full path to map YAML file") 

    namespace = LaunchConfiguration('namespace') 
    use_sim_time = LaunchConfiguration('use_sim_time') 

    localization = GroupAction([
        PushRosNamespace(namespace), 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [pkg_nav2_bringup, 'launch', 'localization_launch.py'])),
                launch_arguments={ 'namespace' : namespace, 
                                'use_sim_time' : use_sim_time, 
                              'map': LaunchConfiguration('map'),
                              'params_file': LaunchConfiguration('params')}.items()),
    ])

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([turtlebot4_viz, 'launch', 'view_robot.launch.py'])),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    navigation_cmd = Node ( 
            package='navigation', 
            executable='navigation', 
            name='navigation_lab', 
            output='screen'
        ) 

    ld = LaunchDescription(ARGUMENTS) 
    ld.add_action(rviz_cmd) 
    ld.add_action(localization_params_arg) 
    ld.add_action(map_arg) 
    ld.add_action(localization) 
    ld.add_action(navigation_cmd) 

    return ld 



    
