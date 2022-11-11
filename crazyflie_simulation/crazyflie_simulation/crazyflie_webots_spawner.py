import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class CrazyflieSpawner:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot

        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_spawner')
        root_node = self.robot.getRoot()
        children_field = root_node.getField('children')
        children_field.importMFNodeFromString(-1, 'Crazyflie {  translation 0 1 -0.05 name "crazyflie"  controller "<extern>"}')

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        #self.node.get_logger().info('step...')

