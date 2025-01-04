'''
该程序通过 BasicNavigator 控制机器人导航到指定的 goal_pose 位置(单点导航)
    在导航过程中获取反馈并输出预计到达时间。
    如果导航任务运行时间超过 600 秒，则取消任务。
    导航结束后，根据结果输出日志，表示导航任务的成功、取消或失败状态。
    通过这个代码，可以控制机器人导航到一个预定义的目标，并通过日志查看任务的进度和结果。
'''

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