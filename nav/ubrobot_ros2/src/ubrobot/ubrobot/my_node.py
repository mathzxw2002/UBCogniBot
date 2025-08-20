import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

from sensor_msgs.msg import JointState  # JointState: ROS标准消息类型，用于描述关节状态（位置、速度、力矩）
from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


class Talker(Node):
    def __init__(self):
        super().__init__('talker_node')
        self.joint_publisher = self.create_publisher(JointState, 'real_joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_real_joint_states)  # 0.02 -> 50 Hz

        #self.robot_config = LeKiwiClientConfig(remote_ip="192.168.0.232", id="my_lekiwi")
        #self.robot = LeKiwiClient(self.robot_config)
        # To connect you already should have this script running on LeKiwi: `python -m lerobot.robots.lekiwi.lekiwi_host --robot.id=my_awesome_kiwi`
        #self.robot.connect()

        print('----------------------------- init success...')

    def publish_real_joint_states(self):
        '''motor_ids = list(self.joint_to_motor_id.values())   # 从字典 self.joint_to_motor_id 中提取所有电机ID
        motor_models = ["sts3215"] * len(motor_ids)         # 创建一个与电机数量相同的列表，所有元素均为电机型号 "sts3215"
        try:
            positions = self.motors_bus.read_with_motor_ids(motor_models, motor_ids, "Present_Position") # 调用 FeetechMotorsBus 的方法，通过电机ID和型号读取当前值
        except (ConnectionError, SerialException) as e:
            self.get_logger().error(f"Connection error: {e}")
            self.get_logger().info("Attempting to reconnect...")
            while True:
                try:
                    self.motors_bus.reconnect()
                    self.get_logger().info("Reconnected to motors.")
                    return
                except Exception as e:
                    self.get_logger().error(f"Reconnection failed: {e}")
                    time.sleep(1)  # Wait for 1 second before retrying'''
 
        #joint_state_msg = JointState()
        #joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        #joint_state_msg.name = "test"#self.motor_names
        #joint_state_msg.position = [((-pos / 4095.0) * (2 * 3.14) - 3.14) for pos in positions]  # Convert motor value to radians
 
        #self.joint_publisher.publish(joint_state_msg)
        #self.get_logger().info(f"Published real joint states: {positions}")
        self.get_logger().info(f"Published real joint states: .....")
        #self.get_logger().info(f"Published real joint states (radians): {joint_state_msg.position}")

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

