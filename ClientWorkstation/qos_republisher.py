#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

from rclpy.executors import ExternalShutdownException

class QoSRepublisher(Node):
    def __init__(self):
        super().__init__('qos_republisher_node')

        # Matches rqt_publisher (Volatile)
        sub_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Matches transient_local subscribers / domain bridge
        pub_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Bridge: /rqt/start_explore -> /JeffCreate3/start_explore
        self.pub_start_explore = self.create_publisher(
            String,
            '/JeffCreate3/start_explore',
            pub_qos
        )
        self.sub_start_explore = self.create_subscription(
            String,
            '/rqt/start_explore',
            self.start_explore_cb,
            sub_qos
        )

        self.get_logger().info("QoS Bridge Active: /rqt/start_explore -> /JeffCreate3/start_explore [Transient Local]")

    def start_explore_cb(self, msg: String):
        self.get_logger().info(f"Relaying command: '{msg.data}' with Transient Local QoS")
        self.pub_start_explore.publish(msg)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = QoSRepublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
