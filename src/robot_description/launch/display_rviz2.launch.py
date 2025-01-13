from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 创建一个参数，用于指定是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 加载URDF文件内容作为一个参数
    pkg_path = get_package_share_directory('robot_description')
    urdf_file_path = os.path.join(pkg_path, 'urdf', 'robot_description.urdf')

    robot_desc = ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)

    return LaunchDescription([
        # 定义命令行参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 启动robot_state_publisher节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }]
        ),

        # 如果需要，可以添加joint_state_publisher或joint_state_publisher_gui
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # 启动rviz2节点，注意这里的缩进，要和其他节点保持一致，包含在LaunchDescription列表内
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
        # 如果需要，这里还可以添加其他节点，例如传感器驱动程序
        # Node(
        #     package='your_sensor_package',
        #     executable='your_sensor_node',
        #     name='your_sensor_node_name',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}]
        # ),
    ])
