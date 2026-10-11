#!/usr/bin/env python3

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.executors import ExternalShutdownException


class HazardToPointCloud(Node):
    def __init__(self):
        super().__init__('hazard_to_pointcloud')

        self.hazard_sub = self.create_subscription(
            HazardDetectionVector,
            'hazard_detection',
            self.hazard_callback,
            qos_profile_sensor_data
        )

        cloud_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.cloud_pub = self.create_publisher(PointCloud2, 'hazard_points', cloud_qos)

        # Create 3 chassis radius is ~0.17m
        self.robot_radius = 0.175

        # Angular mapping relative to base_link (0 rad = front, +pi/2 = left, -pi/2 = right)
        self.hazard_angles = {
            # Bumper zones
            'bump_front_left': np.deg2rad(35.0),
            'bump_front_center': 0.0,
            'bump_front_right': np.deg2rad(-35.0),
            'bump_left': np.deg2rad(75.0),
            'bump_right': np.deg2rad(-75.0),
            # Cliff zones
            'cliff_side_left': np.deg2rad(70.0),
            'cliff_front_left': np.deg2rad(25.0),
            'cliff_front_right': np.deg2rad(-25.0),
            'cliff_side_right': np.deg2rad(-70.0),
        }

        self.get_logger().info('Hazard-to-PointCloud: Accurate base_link obstacle projection initialized.')

    def hazard_callback(self, msg: HazardDetectionVector):
        for detection in msg.detections:
            if detection.type not in (HazardDetection.BUMP, HazardDetection.CLIFF):
                continue

            frame = detection.header.frame_id
            # Strip potential namespace prefixes (e.g. 'JeffCreate3/bump_front_left' -> 'bump_front_left')
            clean_frame = frame.split('/')[-1]

            # Determine central angle along chassis perimeter
            center_angle = self.hazard_angles.get(clean_frame, 0.0)

            is_cliff = (detection.type == HazardDetection.CLIFF)
            hazard_name = "CLIFF" if is_cliff else "BUMP"
            self.get_logger().warn(
                f'[{hazard_name}] on {clean_frame} (~{np.rad2deg(center_angle):.1f}°). Injecting obstacle arc into costmap.'
            )

            # Generate the patch in base_link coordinates
            cloud = self.create_base_link_hazard_patch(
                detection.header.stamp, 
                center_angle=center_angle, 
                is_cliff=is_cliff,
                num_pts=12
            )
            self.cloud_pub.publish(cloud)

    def create_base_link_hazard_patch(self, stamp, center_angle, is_cliff=False, num_pts=12):
        """
        Projects an arc directly outside the robot's physical radius in base_link frame.
        Guarantees the obstacle lands in the costmap without intersecting the robot footprint.
        """
        msg = PointCloud2()
        msg.header.stamp = stamp
        # Publish in base_link (or let ROS namespace prefix apply)
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = num_pts
        msg.is_dense = True
        msg.is_bigendian = False

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 12
        msg.row_step = 12 * num_pts

        byte_data = bytearray()

        # Cliff needs a wider barrier to prevent edge traversal; bump can be tighter
        spread = np.deg2rad(45.0) if is_cliff else np.deg2rad(30.0)
        angles = np.linspace(center_angle - spread / 2, center_angle + spread / 2, num_pts)

        # Stand off slightly past robot radius (radius + 4cm margin)
        standoff_dist = self.robot_radius + 0.04

        for a in angles:
            px = standoff_dist * np.cos(a)
            py = standoff_dist * np.sin(a)
            pz = 0.05  # Height above floor
            byte_data.extend(struct.pack('fff', float(px), float(py), float(pz)))

        msg.data = bytes(byte_data)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = HazardToPointCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()