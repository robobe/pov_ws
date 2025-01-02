import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from pathlib import Path

PLUGINS = 'mavros_plugins.yaml'
CONFIG = "mavros_apm.yaml"

def generate_launch_description():
    bringup_share = get_package_share_directory("pov_bringup")

    pluginlist = os.path.join(
        bringup_share,
        'config',
        PLUGINS
    )

    apm_config = os.path.join(
        bringup_share,
        'config',
        CONFIG
    )

    mavros_launch = (
        Path(bringup_share).joinpath("launch", "mavros.launch").as_posix()
    )
    mavros_runner = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(mavros_launch),
    launch_arguments={
        "pluginlists_yaml": pluginlist,
        "fcu_url": "udp://127.0.0.1:14550@",
        "gcs_url": "",
        "tgt_system": "1",
        "tgt_component": "1",
        "config_yaml": apm_config
    }.items(),
)

    return LaunchDescription(
        [
            mavros_runner
        ]
    )