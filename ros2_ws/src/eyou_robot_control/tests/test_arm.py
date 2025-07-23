# test_arm.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float64MultiArray

class TrajectoryTestClient(Node):
    def __init__(self):
        super().__init__('trajectory_test_client')
        # 确认控制器名称与YAML文件一致
        self._action_client = ActionClient(self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory')

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        # 确保关节名称顺序与控制器配置一致
        trajectory.joint_names = [
            'right_arm_joint_1', 'right_arm_joint_2', 'right_arm_joint_3',
            'right_arm_joint_4', 'right_arm_joint_5', 'right_arm_joint_6', 'right_arm_joint_7'
        ]
        point = JointTrajectoryPoint()
        #point.positions = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 让第一个关节小范围移动
        point.positions = [10.0, 20.0, 10.0, 0.0, 0.0, 0.0, 0.0] # 让第一个关节小范围移动
        point.time_from_start = Duration(sec=2, nanosec=100000000)
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory

        self.get_logger().info('Sending goal to right arm...')
        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal sent.')
        rclpy.shutdown()

def test_trajectory():
    action_client = TrajectoryTestClient()
    action_client.send_goal()

def test_position():
    node = TrajectoryTestClient()
    topic_name = "/right_arm_controller/commands"
    publisher = node.create_publisher(Float64MultiArray, topic_name, 10)
    msg = Float64MultiArray()

        # 3. 填充消息数据
        # -----------------
        # 将 Python 列表赋值给消息的 data 字段。
    msg.data = [10.0, 20.0, 10.0, 0.0, 0.0, 0.0, 0.0]

        # 4. 发布消息
        # -------------
    publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    test_position()
    # 短暂等待确保消息发出
    # 在实际应用中，你会处理 action 的 future 和 result
    # rclpy.spin(action_client)

if __name__ == '__main__':
    main()