"""
Publica el TF estático obtenido por calibración hand-eye.

Conecta el frame óptico de la OAK-D al frame base del UR5 con SoftHand
(brazo I, MoveIt group Right_arm). 
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ur_dual_calibration')
    yaml_path = os.path.join(pkg_share, 'config', 'handeye.yaml')

    with open(yaml_path, 'r') as f:
        cfg = yaml.safe_load(f)['handeye']

    t = cfg['translation']
    q = cfg['rotation_quaternion']

    # static_transform_publisher: x y z qx qy qz qw parent child
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='handeye_static_tf',
        arguments=[
            '--x', str(t['x']),
            '--y', str(t['y']),
            '--z', str(t['z']),
            '--qx', str(q['x']),
            '--qy', str(q['y']),
            '--qz', str(q['z']),
            '--qw', str(q['w']),
            '--frame-id',       cfg['parent_frame'],
            '--child-frame-id', cfg['child_frame'],
        ],
        output='screen',
    )

    return LaunchDescription([static_tf])
