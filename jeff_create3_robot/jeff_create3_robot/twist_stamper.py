#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            qos
        )
        
        self.pub = self.create_publisher(
            TwistStamped,
            'cmd_vel_stamped',
            qos
        )

    def twist_callback(self, msg: Twist):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()