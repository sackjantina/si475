from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 


# This file shows the general setup of how to write a launch file in 
# python: create Nodes for each ROS package you want to run, and add 
# them to a launch description  


def generate_launch_description():
    # store where the package is installed 
    package_dir = get_package_share_directory('localization') 

    # the map server needs a yaml file that provides 
    # information about the map 
    yaml_file = package_dir  + '/map/lab.yaml'

    # this Node is the map server.  The field names 
    # should look familiar.  The parameters array is 
    # one way to pass parameters to a node.  
    map_cmd = Node(
        parameters=[
            {'yaml_filename': yaml_file} , 
        ],
        package='nav2_map_server',
        executable='map_server',
        name='map',
        output='screen')

    # This Node is the rviz.  Note the 
    # arguement field: this is how to pass 
    # command line arguements to a node.  
    rviz_cmd = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(package_dir, 'config', 'localization.rviz')]
        )

    # this is your code.  
    localization_cmd = Node ( 
            package="localization", 
            executable="localization", 
            name="localization", 
            output="screen"
            )

    # We need a transform between the map and base_link to link 
    # everything together.  
    tf_cmd = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]
        )

    # this node provided a model of the turtlebot for rviz.  This is how to 
    # call another ROS package that already has a launch file.  
    description_cmd =  IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory("turtlebot4_description"), '/launch', '/robot_description.launch.py'])
            )


    ld = LaunchDescription()

    # add all the nodes to the launch description.  The underlying 
    # ROS code will handle launching all the nodes 
    ld.add_action(description_cmd) 
    ld.add_action(tf_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(map_cmd)
    ld.add_action(localization_cmd) 

    return ld
