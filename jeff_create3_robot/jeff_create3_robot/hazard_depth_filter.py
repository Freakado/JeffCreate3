#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class DepthDownsamplerNode(Node):
    def __init__(self):
        super().__init__('depth_downsampler_node')

        self.sub = self.create_subscription(
            PointCloud2,
            'realsense2_camera/depth/color/points',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        cloud_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(PointCloud2, 'costmap_cloud', cloud_qos)
        self.leaf_size = 0.05  # 5 cm grid cells

    def cloud_callback(self, msg: PointCloud2):
        # Extract [x, y, z] points
        raw_pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if raw_pts.size == 0:
            return

        x = raw_pts[:, 0]
        y = raw_pts[:, 1]
        z = raw_pts[:, 2]

        # 1. Shorter forward depth window: 0.1m to 1.20m max
        # Depth distortion increases with distance; 1.2m gives plenty of reaction time at 0.2 m/s
        range_mask = (z >= 0.1) & (z <= 1.20)

        # 2. Strict Floor Filter:
        # In optical frame (+Y is DOWN).
        # Reject points anywhere near the lower half of the frame:
        floor_mask = y <= -0.02  # Only accept points above the optical center axis

        valid_mask = range_mask & floor_mask
        if not np.any(valid_mask):
            return

        arr = raw_pts[valid_mask].astype(np.float32)

        # 3. Voxel grid decimation
        coords = np.floor(arr / self.leaf_size).astype(np.int32)
        _, unique_indices = np.unique(coords, axis=0, return_index=True)
        decimated_arr = arr[unique_indices]

        # Pack and publish
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out_msg = pc2.create_cloud(msg.header, fields, decimated_arr)
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthDownsamplerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()