
from launch import LaunchDescription
from launch_ros.actions import Node


#
# Parameters description:
#
# - haarcascades_abs_path_filename: absolut path name of the pre-trained 
#   face detector file, for OpenCV 
#   (you can get one at ht tps://github.com/kipr/opencv/tree/master/data/haarcascades )
#
# - show_viewer: flag to set the visibility of the OpenCV default viewer
#
# - topic_list: the list of the topics names to subscribe
#


def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='jk_demo_msg2cv',
            executable='jk_demo_m2cv_faces_exec',
            #arguments=['--ros-args', '--log-level', 'DEBUG'],
            arguments=['--ros-args', '--log-level', 'INFO'],
            parameters=[{'haarcascades_abs_path_filename': '/home/user/DEV/tools/haarcascade_frontalface_default.xml'},
                        {'show_viewer': True},
                        {'topic_list': ['gst2img_topic_stream_ONE', 'gst2img_topic_stream_TWO']}],
            output='screen',
            emulate_tty=True)  
              
])

