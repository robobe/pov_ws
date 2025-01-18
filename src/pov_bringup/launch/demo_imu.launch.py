from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_file = os.path.join(
        get_package_share_directory('pov_bringup'),
        'config',
        'rviz_imu.rviz'
    )
    #bridge
    # ros2 run ros_gz_bridge parameter_bridge /gz/topic_name@std_msgs/msg/String@std_msgs/msg/String --ros-args --remap /gz/topic_name:=/ros/topic_name
    # '--ros-args',
    #             '--remap',
    #             '/world/Moving_robot/model/vehicle_blue/link/chassis/sensor/imu_sensor/imu:=/imu'
    bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
                
            ]
        )
    
    #bridge
    tf_node = Node(
            package='pov_application',
            executable='world_box_tf.py',
            name='world_box_tf',
            output='screen',
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', config_file],
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(tf_node)
    return ld