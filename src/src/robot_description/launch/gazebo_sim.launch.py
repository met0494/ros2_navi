import launch  # 导入 ROS 2 启动库
import launch_ros  # 导入 ROS 2 的扩展库，提供与 ROS 2 相关的功能
from ament_index_python.packages import get_package_share_directory  # 导入获取软件包共享目录的函数
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入用于包含 Python 启动描述的类
import launch_ros.parameter_descriptions  # 导入处理参数描述的库


def generate_launch_description():
    # 设置机器人模型名称
    robot_name_in_model = "rohbot"

    # 获取 robot_description 包的默认路径
    urdf_tutorial_path = get_package_share_directory('robot_description')

    # 定义默认的机器人模型和世界文件路径
    default_model_path = urdf_tutorial_path + '/urdf/robot.urdf.xacro'
    default_world_path = urdf_tutorial_path + '/world/custom_room.world'

    # 为启动文件声明一个参数，用于指定 URDF 的绝对路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model',  # 参数名称
        default_value=str(default_model_path),  # 默认值为默认模型路径
        description='URDF 的绝对路径'  # 参数描述
    )

    # 获取 xacro 文件内容并生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),  # 执行 xacro 转换
        value_type=str  # 参数类型为字符串
    )

    # 创建机器人状态发布器节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',  # 使用 robot_state_publisher 包
        executable='robot_state_publisher',  # 可执行文件名称
        parameters=[{'robot_description': robot_description}]  # 传递 robot_description 参数
    )

    # 通过 IncludeLaunchDescription 包含 Gazebo 启动文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),  # Gazebo 启动文件的路径
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]  # 传递世界文件路径和 verbose 参数
    )

    # 请求 Gazebo 加载机器人实体
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',  # 使用 gazebo_ros 包
        executable='spawn_entity.py',  # 可执行文件名称
        arguments=['-topic', '/robot_description', '-entity', robot_name_in_model]  # 传递参数，指定描述话题和实体名称
    )

    # 加载并激活 robot_joint_state_broadcaster 关节控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_joint_state_broadcaster'],  # 激活关节状态广播器
        output='screen'  # 输出到屏幕
    )

    # 激活并加载 robot_effort_controller 力控制器
    load_robot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_effort_controller'],  # 激活力控制器
        output='screen'  # 输出到屏幕
    )

    # 激活并加载 robot_diff_drive_controller 两轮差速控制器
    load_robot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_diff_drive_controller'],  # 激活差速驱动控制器
        output='screen'  # 输出到屏幕
    )

    return launch.LaunchDescription([
        # 注册事件处理程序，加载机器人后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,  # 目标是 spawn_entity_node
                on_exit=[load_joint_state_controller]  # 执行 load_joint_state_controller
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,  # 目标是 load_joint_state_controller
                on_exit=[load_robot_effort_controller]  # 执行 load_robot_effort_controller
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_robot_effort_controller,  # 目标是 load_robot_effort_controller
                on_exit=[load_robot_diff_drive_controller]  # 执行 load_robot_diff_drive_controller
            )
        ),
        action_declare_arg_mode_path,  # 添加参数声明
        robot_state_publisher_node,  # 添加机器人状态发布节点
        launch_gazebo,  # 添加 Gazebo 启动文件
        spawn_entity_node  # 添加加载实体的节点
    ])
