import os 
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
launch_ros.actions.Node(
            package='fix_distortion_fisheye_camera',
            executable='t265_node',
            name='t265_node',
            output='screen'
        ),

       
        launch_ros.actions.Node(
            package='fix_distortion_fisheye_camera',
            executable='defisheye_rectilinear_projection.py',  
            output='screen',
            parameters=[],
            arguments=[os.path.join(os.getenv('ROS_WS', '/home/rcasal/ros2_ws'), 'install/fix_distortion_fisheye_camera/lib/fix_distortion_fisheye_camera/defisheye_rectilinear_projection.py')]
        ),
    ])