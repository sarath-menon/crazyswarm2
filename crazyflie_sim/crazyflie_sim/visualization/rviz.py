from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.node import Node


class Visualization:

    def __init__(self, node: Node):
        self.node = node
        self.tfbr = TransformBroadcaster(self.node)

    def init(self, names, positions):
        self.names = names

    def step(self, setpoints):
        # publish transformation to visualize in rviz
        msgs = []
        for name, setpoint in zip(self.names, setpoints):
            msg = TransformStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.child_frame_id = name
            msg.transform.translation.x = setpoint.pos[0]
            msg.transform.translation.y = setpoint.pos[1]
            msg.transform.translation.z = setpoint.pos[2]
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z = 0.0
            msg.transform.rotation.w = 1.0
            msgs.append(msg)
        self.tfbr.sendTransform(msgs)

