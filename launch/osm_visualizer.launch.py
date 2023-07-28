from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="map_visualizer",
            executable="osm_visualizer",
            name="osm_visualizer",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"map_path": "/home/otonom/Downloads/a.osm"}
            ]
        )
    ])