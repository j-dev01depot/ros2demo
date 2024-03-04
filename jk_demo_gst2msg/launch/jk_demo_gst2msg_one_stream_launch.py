
from launch import LaunchDescription
from launch_ros.actions import Node


#
# Parameters description:
#
# video_abs_path_filename: absolut path name of video file to use
#
# milliseconds: time interval in milliseconds between topic publications
#
# show_viewer: flag to set the visibility of the OpenCV default viewer
#


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jk_demo_gst2msg',
            executable='jk_demo_gst2m_exec', 
            # arguments=['--ros-args', '--log-level', 'DEBUG'],
            arguments=['--ros-args', '--log-level', 'INFO'],
            name='gstr2imgmsg_node',
            remappings=[('gst2img_topic', 'gst2img_topic_stream')],
            output='screen',
            emulate_tty=True,
            parameters=[{'video_abs_path_filename': '/home/user/DEV/tools/EG_360_videoONE.mp4'},
                        {'milliseconds': 2000},
                        {'show_viewer': True}])
  ])
  
