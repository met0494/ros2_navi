import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取默认路径
    robot_name_in_model = "robot"
    urdf_tutorial_path = get_package_share_directory('robot_description')
    default_model_path = urdf_tutorial_path + '/urdf/robot_description.urdf'
    default_world_path = '/usr/share/gazebo-11/worlds/empty.world'  # urdf_tutorial_path + '/world/custom_room.world'
    
    #为 launch 声明参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径'
    )
    
    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher', 
        parameters=[{'robot_description':robot_description}]
    )
    
    # 通过 IncludeLaunchDescription 包含另外一个 launch 文件
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
            # 传递参数
        launch_arguments=[('world', default_world_path), ('verbose', 'true')]
    )
    
    # 请求 gazebo 加载机器人
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name_in_model, ]
    )

    # 加载并激活 robot_joint_state_broadcaster 关节控制器 的动作定义
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_joint_state_broadcaster'],
        output='screen'
    )

    # 激活并加载 robot_effort_controller 力控制器 的动作定义
    load_robot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_effort_controller'],
        output='screen'
    )

    # 激活并加载 robot_diff_drive_controller 两轮差速控制器 的动作定义
    load_robot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'robot_diff_drive_controller'],
        output='screen'
    )

    return launch.LaunchDescription([
        # 事件动作，当加载机器人结束后执行
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,    # 当spawn_entity_node执行完毕后
                on_exit=[load_joint_state_controller], # 执行load_joint_state_controller
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,  # 当load_joint_state_controller执行完毕后
                on_exit=[load_robot_effort_controller],   # 执行load_robot_effort_controller
            )
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_robot_effort_controller,   # 当load_robot_effort_controller执行完毕后
                on_exit=[load_robot_diff_drive_controller],   # 执行load_robot_diff_drive_controller
            )
        ),
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])
