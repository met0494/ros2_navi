'''
通过 launch 和 launch_ros 模块来启动 ROS 2 节点和配置导航系统
主要作用是启动 ROS 2 的 Nav2 导航栈，同时加载 RViz 进行可视化
'''

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():  # ROS 2 启动文件的入口点，它返回一个 LaunchDescription 对象，其中包含了要启动的节点、配置、参数等
    # 获取与拼接默认路径
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    '''
    get_package_share_directory：分别获取两个功能包的共享目录：
        robot_navigation2：用于获取地图、配置等相关文件。
        nav2_bringup：这是 Nav2 的启动包，它包含了启动导航功能所需的脚本和配置文件。
    '''
    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )   # 使用 os.path.join 拼接出 RViz 配置文件的完整路径，这个配置文件用于控制 RViz 的界面和显示内容

    # 创建 launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    # use_sim_time：定义一个 Launch 配置参数，默认为 'true'，表示使用仿真时间（如 Gazebo 时间）。这个参数会被传递给启动的节点，告诉它们使用仿真时间而不是系统的真实时间
    map_yaml_path = launch.substitutions.LaunchConfiguration('map', default = os.path.join(robot_navigation2_dir, 'maps', 'room.yaml'))
    # map_yaml_path：定义地图文件的路径，这里默认值是 robot_navigation2 包中的 room.yaml 文件
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file', default = os.path.join(robot_navigation2_dir, 'config', 'nav2_params.yaml'))
    # nav2_param_path：定义导航参数文件的路径，默认为 robot_navigation2 包中的 nav2_params.yaml

    '''
    DeclareLaunchArgument：定义了可以从外部传递的参数，这些参数可以在启动时被用户覆盖。
    如果用户没有提供值，就会使用这里定义的默认值
        'use_sim_time'：是否使用仿真时间。
        'map'：地图文件路径。
        'params_file'：导航参数文件路径
    '''
    return launch.LaunchDescription([   
        # 声明新的 launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', 
                                             default_value=use_sim_time,
                                             description='Use simulation (Gazebo) clock if true'
                                             ),
        launch.actions.DeclareLaunchArgument('map', 
                                             default_value=map_yaml_path,
                                             description='Full path to map file to load'
                                             ),
        launch.actions.DeclareLaunchArgument('params_file', 
                                             default_value=nav2_param_path,
                                             description='Full path to param file to load'
                                             ),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 launch 参数替换原本参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # IncludeLaunchDescription：包含另一个启动文件，这里包含了 Nav2 自带的 bringup_launch.py 文件。这个文件负责启动导航系统
        # launch_arguments：将上面定义的 map、use_sim_time 和 params_file 参数传递给 bringup_launch.py 文件，这些参数会覆盖 bringup_launch.py 中的默认值
        launch_ros.actions.Node(
            package='rviz2',    # 指定功能包名称为 rviz2
            executable='rviz2', # 指定可执行文件为 rviz2，启动 RViz 应用
            name='rviz2',   
            arguments=['-d', rviz_config_dir],  # 指定 RViz 启动时加载的配置文件，这里使用之前获取的 nav2_default_view.rviz 文件
            parameters=[{'use_sim_time': use_sim_time}],    # 传递参数 use_sim_time，告知 RViz 是否使用仿真时间
            output='screen' # 将节点的输出打印到终端屏幕上
        ),
    ])