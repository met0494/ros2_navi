# readme

## 文件结构树以及简单注解

─ /src
├ /robot_description	# 导航小车的描述文件
│	├ /config
│	│	└ robot_ros2_controller.yaml	# 定义用于机器人控制的关节状态广播器、力控制器、以及差速驱动控制器的配置
│	├ /include
│	│	└ /robot_description
│	├ /launch
│	│	└ gazebo_sim.launch.py	# 加载机器人模型、启动Gazebo仿真环境并初始化相关控制器的仿真启动脚本
│	├ /src
│	├ /urdf
│	│	├ base.urdf.xacro	# 定义机器人基础部分的URDF描述
│	│	├ common_inertia.xacro	# 机器人惯性参数计算URDF描述文件
│	│	├ robot.ros2_control.xacro	# 整合ROS 2控制插件和Gazebo仿真插件
│	│	├ robot.urdf.xacro	# 整合机器人各个组件的URDF文件
│	│	├ /actuator
│	│	│	├ caster.urdf.xacro	# 定义机器人万向轮的URDF描述
│	│	│	└ wheel.urdf.xacro	# 定义机器人的驱动轮组件
│	│	├ /plugins
│	│	│	├ gazebo_control_plugin.xacro	# 定义一个用于控制机器人差速驱动的Gazebo插件宏
│	│	│	└ gazebo_sensor_plugin.xacro	# 定义不同激光雷达、深度相机和IMU的Gazebo插件
│	│	└ /sensor
│	│		  ├ camera.urdf.xacro	# 定义机器人相机模块的URDF描述
│	│		  ├ imu.urdf.xacro	# 定义用于仿真环境中的IMU模块（惯性测量单元）的描述
│	│		  └ laser.urdf.xacro	# 定义机器人激光雷达模块的URDF描述
│	├ /world
│	│	├ custom_room.world	# 环境定义的代码描述
│	│	└ /room
│	│		  ├ model.config	# 使用SDF格式描述机器人模型的基础信息
│	│		  └ model.sdf	# 使用SDF格式描述机器人仿真环境模型
│	├ CMakeList.txt	# ROS 2中用于构建和安装机器人描述包的CMake配置文件
│	└ package.xml	# 用于ROS 2系统中的包配置文件
│
│
├ /robot_navigation2	# 导航小车的导航算法文件
│	├ /config
│	│	└ nav2_params.yaml	# 用于导航堆栈（Nav2）的参数配置文件，包含了多个节点的运行参数，用于实现机器人导航功能
│	├ /include
│	│	└ /fishbot_navigation2
│	├ /launch
│	│	└ navigation2.launch.py	# 启动导航系统（Nav2）并加载 RViz 进行可视化
│	├ /maps
│	│	├ room.pgm
│	│	└ room.yaml	# 用于加载地图的配置文件，ROS 2通过该文件加载环境地图
│	├ /src
│	├ CMakeList.txt	# 遵循ament构建系统的标准，定义了项目的基本信息、依赖项的查找以及测试框架的配置
│	└ package.xml	# ROS 2 包描述文件
│
│
└ /robot_application	# 导航小车的实际应用层文件
  	├ /resource
  	│	└ robot_application
  	├ /robot_application
  	│	├ init.py
  	│	├ get_robot_pose.py	# 通过监听ROS 2中的TF（坐标变换）数据，实时获取机器人在地图中的位置信息和朝向（旋转）
  	│	├ init_robot_pose.py	# 通过 nav2_simple_commander 设置机器人初始位姿，并启动导航功能
  	│	├ nav_to_pose.py	# 单点导航程序，通过 BasicNavigator 控制机器人导航到指定的目标位置 (goal_pose)
  	│	└ waypoint_follower.py	# 多点导航任务的机器人控制程序
  	├ /test
  	│	├ test_copyright.py
  	│	├ test_flake8.py
  	│	└ test_pep257.py
  	├ package.xml	# 用于ROS 2系统中的包配置文件
  	├ setup.cfg	# 用于配置ROS 2项目的安装和开发环境，特别是指定了脚本的开发目录和安装路径
  	└ setup.py	# 用于配置ROS 2下Python包的安装和分发的setup.py文件

## 项目启动以及配置方式

### 项目启动

打开新终端，先进入项目的工作空间srp_ws

```powershell
colcon build	# 编译项目代码
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 launch robot_description gazebo_sim.launch.py	# 加载机器人模型、启动Gazebo仿真环境并初始化相关控制器
```

再打开一个新终端，进入项目的工作空间srp_ws

```powershell
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 launch robot_navigation2 navigation2.launch.py	# 启动导航系统（Nav2）并加载 RViz 进行可视化
```

最后打开一个新终端，进入项目的工作空间srp_ws

```powershell
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 run robot_application init_robot_pose	# 设置机器人初始位姿，并启动导航功能
```

#### 手动操作

打开一个新终端，进入项目的工作空间srp_ws

```powershell
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 run teleop_twist_keyboard teleop_twist_keyboard	# 启动仿真机器人控制器，控制机器人的移动
```

#### 单点自动导航

打开一个新终端，进入项目的工作空间srp_ws

```powershell
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 run robot_application nav_to_pose	# 启动引导机器人自动抵达设定好的单点的控制脚本
```

#### 多点自动巡航

打开一个新终端，进入项目的工作空间srp_ws

```powershell
source install/setup.bash	# 将项目添加到当前终端的根路径
ros2 run robot_application waypoint_follower	# 启动引导机器人自动抵达设定好的多个点的控制脚本
```

### 项目配置的修改

#### 单点导航的修改

/src/robot_application/robot_application/nav_to_pose.py文件：

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main(): # 定义程序的主函数，用于启动导航任
    rclpy.init()    # 初始化 ROS 2 客户端，准备创建和使用 ROS 2 节点
    navigator = BasicNavigator()    # 创建 BasicNavigator 对象，控制机器人导航
    navigator.waitUntilNav2Active() # 等待 Nav2（导航栈）进入活动状态。这确保导航系统中的所有必要节点（如地图服务器、局部和全局规划器）都已准备好工作
    goal_pose = PoseStamped()   # 创建一个 PoseStamped 对象，表示机器人导航的目标位姿
    goal_pose.header.frame_id='map'  # 设置目标位姿的参考坐标系为 map，表示目标点是在全局地图中定义的
    goal_pose.header.stamp=navigator.get_clock().now().to_msg() # 设置时间戳为当前时间
    goal_pose.pose.position.x=1.0
    goal_pose.pose.position.y=1.0   # 定义目标点的坐标为 (1.0, 1.0)，即机器人需要导航到的地图位置
    goal_pose.pose.orientation.w=1.0    # 表示机器人没有旋转
    navigator.goToPose(goal_pose)   # 开始导航到指定的 goal_pose（目标点）
    while not navigator.isTaskComplete():   # 进入一个循环，直到导航任务完成
        feedback = navigator.getFeedback()  # 获取导航过程中的反馈信息（如剩余时间等）
        navigator.get_logger().info(    # 输出日志，显示导航任务剩余的预计时间
            f'预计：{Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达'
        )   # Duration.from_msg 将反馈中的 estimated_time_remaining 转换为秒数，并将结果以秒为单位打印出来
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):  
            navigator.cancelTask()  # 检查导航任务的执行时间，如果超过 600 秒，则取消导航任务
    result = navigator.getResult()  # 获取导航任务的结果状态（成功、失败、被取消等）
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('导航结果：被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('导航结果：失败')
    else:
        navigator.get_logger().error('导航结果：返回状态无效')
```

通过修改：

```python
goal_pose.pose.position.x=1.0
goal_pose.pose.position.y=1.0   # 定义目标点的坐标为 (1.0, 1.0)，即机器人需要导航到的地图位置
```

即可修改单点导航的目的地

#### 多点巡航的修改

/src/robot_application/robot_application/waypoint_follower.py文件：

```python
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def main(): # 定义程序的主函数，用于启动导航任
    rclpy.init()    # 初始化 ROS 2 客户端，准备创建和使用 ROS 2 节点
    navigator = BasicNavigator()    # 创建 BasicNavigator 对象，控制机器人导航
    navigator.waitUntilNav2Active() # 等待 Nav2（导航栈）进入活动状态。这确保导航系统中的所有必要节点（如地图服务器、局部和全局规划器）都已准备好工作
    goal_poses = [] # 创建一个空列表 goal_poses，用于存储多个导航点

    goal_pose1 = PoseStamped()  # 创建第一个导航点，使用 PoseStamped 消息类型
    goal_pose1.header.frame_id='map'    # 设置参考坐标系为 map（表示全局地图坐标系）
    goal_pose1.header.stamp=navigator.get_clock().now().to_msg()    # 获取当前时间戳，赋值给导航点的时间戳
    goal_pose1.pose.position.x=0.0  # 设置第一个导航点的 x 坐标为 0.0
    goal_pose1.pose.position.y=0.0  # 设置第一个导航点的 y 坐标为 0.0
    goal_pose1.pose.orientation.w=1.0   # 四元数的 w=1.0 表示没有旋转（朝向正北）
    goal_poses.append(goal_pose1)   # 将第一个导航点添加到 goal_poses 列表中

    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id='map'
    goal_pose2.header.stamp=navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x=2.0
    goal_pose2.pose.position.y=0.0
    goal_pose2.pose.orientation.w=1.0
    goal_poses.append(goal_pose2)

    goal_pose3 = PoseStamped()
    goal_pose3.header.frame_id='map'
    goal_pose3.header.stamp=navigator.get_clock().now().to_msg()
    goal_pose3.pose.position.x=2.0
    goal_pose3.pose.position.y=2.0
    goal_pose3.pose.orientation.w=1.0
    goal_poses.append(goal_pose3)

    navigator.followWaypoints(goal_poses)   # 调用 followWaypoints() 方法，传入导航点列表 goal_poses，开始让机器人依次导航到这些点
    while not navigator.isTaskComplete():   # 进入一个循环，直到导航任务完成
        feedback = navigator.getFeedback()  # 获取导航过程中的反馈信息（如剩余时间等）
        navigator.get_logger().info(f'当前目标编号：{feedback.current_waypoint}')   # 输出日志，显示机器人当前正在前往的导航点编号
    result = navigator.getResult()  # 获取导航任务的结果状态（成功、失败、被取消等） 
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('导航结果：成功')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('导航结果：被取消')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('导航结果：失败')
    else:
        navigator.get_logger().error('导航结果：返回状态无效')

```

通过修改或增加或减少代码块：

```python
goal_pose1 = PoseStamped()  # 创建第一个导航点，使用 PoseStamped 消息类型
goal_pose1.header.frame_id='map'    # 设置参考坐标系为 map（表示全局地图坐标系）
goal_pose1.header.stamp=navigator.get_clock().now().to_msg()    # 获取当前时间戳，赋值给导航点的时间戳
goal_pose1.pose.position.x=0.0  # 设置第一个导航点的 x 坐标为 0.0
goal_pose1.pose.position.y=0.0  # 设置第一个导航点的 y 坐标为 0.0
goal_pose1.pose.orientation.w=1.0   # 四元数的 w=1.0 表示没有旋转（朝向正北）
goal_poses.append(goal_pose1)   # 将第一个导航点添加到 goal_poses 列表中
```

即可修改多点巡航每个点的目的地以及目的地数量