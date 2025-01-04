'''
通过这个程序，你可以在终端看到机器人在地图中的位置信息和朝向（旋转）
'''

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    '''
    该类用于监听 TF
    '''
    def __init__(self):
        super().__init__('tf2_listener')    # 调用父类 Node 的构造函数，并将节点命名为 'tf2_listener'。这意味着这个 ROS 2 节点的名称是 tf2_listener
        self.buffer = Buffer()  # 创建一个 TF 缓存 Buffer 对象，用来存储坐标变换信息
        self.listener = TransformListener(self.buffer, self)    # 创建一个 TransformListener 对象，将其绑定到 self.buffer 上，这样 TF 数据就会被自动填充到缓存中。self 作为节点传递给监听器
        self.timer = self.create_timer(1, self.get_transform)   # 创建一个定时器，每秒钟触发一次（间隔 1 秒）。定时器触发时会调用 self.get_transform 方法
    
    def get_transform(self):
        '''
        定义一个方法 get_transform，用于获取从 map 到 base_footprint 的坐标变换
        '''
        try:
            '''
            调用 lookup_transform 从缓存中查找 map 坐标系到 base_footprint 坐标系的最新坐标变换。rclpy.time.Time(seconds=0) 表示获取最新的变换数据。
            rclpy.time.Duration(seconds=1) 是超时时间，表示如果在 1 秒内没有找到变换，则抛出异常。
            '''
            tf = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = tf.transform    # 提取坐标变换的 transform 数据部分，它包含平移和旋转信息。
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w])  # 使用 euler_from_quaternion 将 transform.rotation 中的四元数旋转信息转换为欧拉角（roll、pitch、yaw）。四元数 [x, y, z, w] 转换为欧拉角
            self.get_logger().info(f'平移:{transform.translation},旋转四元数:{transform.rotation},旋转欧拉角:{rotation_euler}') # 通过节点的日志系统输出变换数据
        except Exception as e:  # 捕获所有异常，如果出现问题（如无法找到坐标变换），则输出警告。
            self.get_logger().warn(f'不能够获取坐标变换，原因:{str(e)}')

def main(): # 定义 main() 函数，作为 ROS 2 节点的入口
    rclpy.init()        # 初始化 ROS 2 客户端，这一步必须在使用 ROS 2 功能之前调用，启动 ROS 2 节点管理系统
    node = TFListener() # 创建 TFListener 节点对象，该节点将开始监听 TF 并处理数据
    rclpy.spin(node)    # 进入 ROS 2 事件循环，保持节点运行并监听消息和事件。它会一直运行，直到节点被手动关闭或发生中断
    rclpy.shutdown()    # 停止 ROS 2 节点并清理资源，在程序退出时调用。