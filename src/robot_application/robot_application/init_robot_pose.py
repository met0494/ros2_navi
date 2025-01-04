'''
这段代码的主要功能是在 ROS 2 中通过 nav2_simple_commander 设置机器人初始位姿，并启动导航功能。具体流程如下：
    初始化 ROS 2 系统：通过 rclpy.init() 启动 ROS 2 节点。
    创建导航器对象：利用 BasicNavigator 控制导航功能。
    设置初始位姿：通过 PoseStamped 指定机器人在地图中的初始位置和方向（在原点，且无旋转）。
    等待导航系统激活：确保 Nav2（导航堆栈）完全启动并准备好工作。
    保持程序运行：使用 rclpy.spin() 让节点持续运行并处理消息。
    关闭节点：在程序结束时调用 rclpy.shutdown() 释放系统资源。
    这个程序的核心是初始化导航器，设置机器人位姿，并确保导航系统可用。
'''

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
def main():
    rclpy.init()    # 创建一个 ROS2 客户端节点，使得可以与 ROS 2 网络进行交互
    navigator = BasicNavigator()    # 创建 BasicNavigator 对象，navigator 代表一个导航器，它提供了接口与 ROS 2 的 Nav2（导航栈）进行交互，允许设置目标、监控导航状态、获取机器人位置等
    initial_pose = PoseStamped()    # 创建一个 PoseStamped 对象，用于设置机器人在地图中的初始位置和方向。PoseStamped 是带有时间戳和参考坐标系的位姿信息，它将在稍后用来设置机器人的初始位置
    initial_pose.header.frame_id = 'map'    # 设置该位姿的参考坐标系为 map，这意味着这个位姿是相对于全局地图的。frame_id 定义了位姿所属的坐标系，在这里是全局地图坐标系
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()    # 为位姿添加时间戳，表示这个位置在什么时间被设置。navigator.get_clock().now().to_msg() 返回当前时间，并转换为适合 ROS 消息格式的时间戳。时间戳对于时间敏感的操作（如同步多个传感器数据）非常重要。
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0  # 定义了机器人在地图中的初始位置坐标，在这里设置为原点 (0.0, 0.0)
    initial_pose.pose.orientation.w = 1.0   # orientation.w：四元数表示的姿态（方向），这里设置为 1.0，表示机器人没有旋转，即面向正方向。这个四元数的值表示机器人的旋转部分，四元数 (0, 0, 0, 1) 表示没有旋转
    navigator.setInitialPose(initial_pose)  # 使用 setInitialPose() 方法将机器人设置到初始位置。这会告诉导航栈机器人从这个初始位姿开始进行导航
    navigator.waitUntilNav2Active() # 调用 waitUntilNav2Active() 方法，阻塞程序执行，直到导航系统（Nav2）变为活动状态。该方法确保导航堆栈中的所有必要节点（如地图服务器、局部规划器等）启动并准备好接收导航目标
    rclpy.spin(navigator)   # 启动 ROS 2 节点的事件循环，保持程序运行，并允许处理传入的 ROS 消息和回调函数。spin() 方法会阻塞并让节点处于活动状态，从而监听订阅消息、处理回调等操作
    rclpy.shutdown()    # 在节点运行结束后，调用 rclpy.shutdown() 来清理资源并安全地关闭 ROS 2 通信系统。这是 ROS 2 节点退出时的标准操作，确保系统资源不会被泄露。