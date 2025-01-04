'''
通过 ROS 2 中的 nav2_simple_commander 库，实现了一个多点导航的机器人任务
定义了一个机器人依次到达多个导航点（waypoints）的过程，并反馈每个导航点的状态
'''

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
