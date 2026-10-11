#!/usr/bin/env python3

import json
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from tf_transformations import quaternion_from_euler
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped
from pyzbar.pyzbar import decode as decode_qr

from urllib.parse import urlparse, parse_qs

TAG_FAMILY = "tag36h11"
TAG_SIZE_METERS = 0.08             # Physical size of printed AprilTag (meters)

# --- Tunable Parameters ---
MAX_INITIAL_DETECTION_DIST_M = 2.00  # Reject distant detections
STANDOFF_DISTANCE_M          = 0.60  # Safe standoff outside the wall inflation footprint
MAX_SCAN_DISTANCE_M          = 0.80  # Optical cutoff for QR decoding
REQUIRED_INITIAL_SAMPLES     = 5     # Consensus for approach dispatch

INSPECTION_SETTLE_FRAMES = 40
MAX_QR_RETRIES = 3
NAV_TIMEOUT_SEC = 3.0

IMAGE_CENTER_DEADBAND_PX = 60
ALIGN_ANGULAR_SPEED = 0.20


class ArtifactInspectorNode(Node):
    def __init__(self):
        super().__init__('artifact_inspector_node')

        self.declare_parameter('bypass_nav', False)
        self.declare_parameter('tag_family', TAG_FAMILY)
        self.declare_parameter('tag_size', TAG_SIZE_METERS)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('standalone_test_mode', False)
        self.declare_parameter('puzzle_wait_timeout_sec', 10.0)

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.standalone_test_mode = self.get_parameter('standalone_test_mode').get_parameter_value().bool_value
        self.bypass_nav = self.get_parameter('bypass_nav').value
        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = self.get_parameter('tag_size').value
        self.puzzle_wait_timeout = self.get_parameter('puzzle_wait_timeout_sec').get_parameter_value().double_value

        if self.standalone_test_mode or self.bypass_nav:
            self.get_logger().warn(
                f"[ArtifactInspector] Standalone/Bypass Active: target_frame='{self.target_frame}', bypass_nav={self.bypass_nav}"
            )


        self.bridge = CvBridge()
        self.detector = Detector(
            families=self.tag_family,
            nthreads=2,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25
        )

        # State Tracking
        self.camera_params = None
        self.image_width = 640
        self.discovered_tags = set()
        self.solved_artifacts = set()
        
        self.active_inspection = False
        self.navigating_to_standoff = False
        self.aligning_camera = False
        self.inspecting_qr = False
        self.awaiting_help = False
        self.waiting_for_puzzle = False
        self.puzzle_wait_start_time = None
        self.puzzle_timeout_notified = False

        
        self.current_tag_id = None
        self.current_tag_pose_target = (0.0, 0.0)
        self.current_tag_pose_stamped = None
        self.qr_search_counter = 0
        self.qr_retry_counter = 0
        self.has_nav_lock = False

        # Multi-frame filtering buffer
        self.tag_pose_buffers = {}
        self.REQUIRED_SAMPLES = REQUIRED_INITIAL_SAMPLES

        # Diagnostic Counters
        self.frame_count = 0

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # QoS
        sensor_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT, durability=QoSDurabilityPolicy.VOLATILE)
        latched_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        operator_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE)
        marker_qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.request_nav_pub = self.create_publisher(String, 'request_nav2', operator_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.artifact_pub = self.create_publisher(String, 'discovered_artifact', operator_qos)
        self.marker_pub = self.create_publisher(MarkerArray, 'artifact_markers', marker_qos)
        self.help_pub = self.create_publisher(String, 'robot_help', operator_qos)
        self.explore_ctrl_pub = self.create_publisher(String, 'start_explore', latched_qos)
        
        # Nav2 Client (Only used when bypass_nav is False)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscriptions
        self.active_nav_sub = self.create_subscription(String, 'active_nav2', self.active_nav_callback, latched_qos)
        self.cam_info_sub = self.create_subscription(CameraInfo, 'realsense2_camera/color/camera_info', self.camera_info_callback, sensor_qos)
        self.image_sub = self.create_subscription(Image, 'realsense2_camera/color/image_raw', self.image_callback, sensor_qos)
        self.clear_help_sub = self.create_subscription(String, 'clear_help', self.clear_help_callback, operator_qos)
        self.puzzle_solved_sub = self.create_subscription(String, 'puzzle_solved', self.puzzle_solved_callback, operator_qos)

        # Diagnostic Heartbeat
        self.diag_timer = self.create_timer(3.0, self.log_diagnostics)
        self.get_logger().info(f"Inspector started (family='{self.tag_family}', tag_size={self.tag_size}m).")

    def transform_pose_with_fallback(self, source_pose: PoseStamped):
        """Transforms PoseStamped into target_frame with odom and base_link fallbacks."""
        frames = [self.target_frame, 'odom', 'base_link']
        for f in frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    f,
                    source_pose.header.frame_id,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                transformed = do_transform_pose_stamped(source_pose, transform)
                return transformed, f
            except TransformException:
                continue
        return None, None

    def log_diagnostics(self):
        state_str = "DISCOVERY"
        if self.awaiting_help:
            state_str = "AWAITING_HELP"
        elif self.waiting_for_puzzle:
            state_str = "WAITING_FOR_PUZZLE"
        elif self.inspecting_qr:
            state_str = "INSPECTING_QR"
        elif self.aligning_camera:
            state_str = "ALIGNING_CAMERA"
        elif self.navigating_to_standoff:
            state_str = "NAVIGATING_STANDOFF"
        elif self.active_inspection:
            state_str = "ACCUMULATING_SAMPLES"

        cam_ok = "READY" if self.camera_params is not None else "WAITING_CAMERA_INFO"
        self.get_logger().info(
            f"[Diag] State: {state_str} | Camera: {cam_ok} | Frames Processed: {self.frame_count} | "
            f"Discovered: {list(self.discovered_tags)} | Solved: {list(self.solved_artifacts)}"
        )

        # manage wait for solution, notify user. 
        if self.waiting_for_puzzle and self.puzzle_wait_start_time is not None:
            elapsed_sec = (self.get_clock().now() - self.puzzle_wait_start_time).nanoseconds / 1e9

            if elapsed_sec > self.puzzle_wait_timeout and not self.puzzle_timeout_notified:
                self.puzzle_timeout_notified = True
                self.trigger_operator_alert(self.current_tag_id, elapsed_sec)

    def trigger_operator_alert(self, tag_id: int, elapsed_sec: float):
            alert_msg = {
                "status": "ATTENTION_REQUIRED",
                "reason": "PUZZLE_WAIT_TIMEOUT",
                "tag_id": tag_id,
                "elapsed_sec": round(elapsed_sec, 1),
                "inspect_url": f"http://192.168.1.100:8000/inspect?tag={tag_id}",
                "message": f"Robot holding for Tag #{tag_id} over {elapsed_sec:.0f}s. Operator intervention needed."
            }
            self.help_pub.publish(String(data=json.dumps(alert_msg)))
            self.get_logger().warn(f"🔔 [TIMEOUT ALERT] Puzzle wait exceeded {elapsed_sec:.1f}s for Tag #{tag_id}!")

            # In standalone test mode, skip automatically to prevent blocking bench runs
            if self.standalone_test_mode:
                self.get_logger().warn(f"[Standalone Test] Auto-skipping Tag #{tag_id} to continue pipeline.")
                self.resume_exploration()

    def active_nav_callback(self, msg: String):
        self.has_nav_lock = (msg.data.strip().lower() == "inspector")

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_params is None:
            self.camera_params = [msg.k[0], msg.k[4], msg.k[2], msg.k[5]]
            self.image_width = msg.width
            self.get_logger().info(f"Camera intrinsics received: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}, cx={msg.k[2]:.1f}, cy={msg.k[5]:.1f}")

    def clear_help_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == "retry":
            self.awaiting_help = False
            self.inspecting_qr = True
            self.qr_search_counter = 0
            self.qr_retry_counter = 0
        elif command in ("skip", "resume"):
            if self.current_tag_id is not None:
                self.discovered_tags.add(self.current_tag_id)
            self.resume_exploration()

    def puzzle_solved_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            tag_id = data.get("tag_id")
            if tag_id is not None:
                self.solved_artifacts.add(tag_id)
                if self.current_tag_pose_stamped:
                    self.publish_markers(tag_id, self.current_tag_pose_stamped, "[SOLVED]", is_solved=True)
                if self.active_inspection and self.current_tag_id == tag_id:
                    self.resume_exploration()
        except Exception as ex:
            self.get_logger().error(f"Error handling puzzle_solved: {ex}")

    def image_callback(self, msg: Image):
        self.frame_count += 1

        if self.camera_params is None:
            if self.frame_count % 30 == 0:
                self.get_logger().warn("Waiting for realsense2_camera/color/camera_info topic...")
            return

        if self.awaiting_help:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CvBridge decode error: {e}", throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # State 1: Discovery & Accumulation
        if not self.active_inspection:
            try:
                detections = self.detector.detect(
                    gray, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size
                )
            except Exception:
                return

            valid = []
            for d in detections:
                dist = float(np.linalg.norm(d.pose_t))
                if dist > MAX_INITIAL_DETECTION_DIST_M:
                    continue
                if d.tag_id not in self.discovered_tags and d.tag_id not in self.solved_artifacts:
                    valid.append(d)

            if valid:
                target = min(valid, key=lambda d: float(np.linalg.norm(d.pose_t)))
                self.accumulate_tag_observation(target, msg.header)

        # State 2: Visual Alignment
        elif self.aligning_camera:
            self.execute_visual_servoing(gray)

        # State 3: Close-Range QR Decoding
        elif self.inspecting_qr:
            self.inspect_qr_payload(cv_image)

    def accumulate_tag_observation(self, detection, header):
        tag_id = detection.tag_id

        tag_cam_pose = PoseStamped()
        tag_cam_pose.header.frame_id = header.frame_id
        tag_cam_pose.header.stamp = rclpy.time.Time().to_msg()
        tag_cam_pose.pose.position.x = float(detection.pose_t[0])
        tag_cam_pose.pose.position.y = float(detection.pose_t[1])
        tag_cam_pose.pose.position.z = float(detection.pose_t[2])
        tag_cam_pose.pose.orientation.w = 1.0

        tag_target_pose, resolved_frame = self.transform_pose_with_fallback(tag_cam_pose)
        if tag_target_pose is None:
            self.get_logger().warn(
                f"TF transform {self.target_frame} <- {header.frame_id} failed. Check TF tree.",
                throttle_duration_sec=2.0
            )
            return

        if tag_id not in self.tag_pose_buffers:
            self.tag_pose_buffers[tag_id] = []
            self.explore_ctrl_pub.publish(String(data='stop'))
            self.get_logger().info(f"Targeting Tag #{tag_id} in '{resolved_frame}'. Gathering {self.REQUIRED_SAMPLES} samples...")

        self.tag_pose_buffers[tag_id].append((
            tag_target_pose.pose.position.x,
            tag_target_pose.pose.position.y,
            tag_target_pose.pose.position.z
        ))

        if len(self.tag_pose_buffers[tag_id]) >= self.REQUIRED_SAMPLES:
            samples = np.array(self.tag_pose_buffers[tag_id])
            median_pos = np.median(samples, axis=0)
            std_dev = np.std(samples, axis=0)

            self.get_logger().info(
                f"Tag #{tag_id} pose locked at [{median_pos[0]:.2f}, {median_pos[1]:.2f}] (frame={resolved_frame}, σ={np.max(std_dev):.3f}m)"
            )

            consensus_pose = PoseStamped()
            consensus_pose.header.frame_id = resolved_frame
            consensus_pose.header.stamp = self.get_clock().now().to_msg()
            consensus_pose.pose.position.x = float(median_pos[0])
            consensus_pose.pose.position.y = float(median_pos[1])
            consensus_pose.pose.position.z = float(median_pos[2])
            consensus_pose.pose.orientation.w = 1.0

            self.tag_pose_buffers.clear()
            self.start_standoff_approach(tag_id, consensus_pose)

    def start_standoff_approach(self, tag_id: int, tag_pose: PoseStamped):
        self.active_inspection = True
        self.current_tag_id = tag_id
        self.current_tag_pose_stamped = tag_pose
        self.current_tag_pose_target = (tag_pose.pose.position.x, tag_pose.pose.position.y)

        self.request_nav_pub.publish(String(data='inspector'))
        self.publish_markers(tag_id, tag_pose, "[APPROACHING]", is_solved=False)

        # Bypass Nav Mode (Bench / Isolated testing)
        if self.bypass_nav:
            self.get_logger().info(f"[Bypass Nav] Skipping motion. Transitioning directly to alignment/QR scan.")
            self.aligning_camera = True
            return

        ref_frame = tag_pose.header.frame_id
        try:
            robot_tf = self.tf_buffer.lookup_transform(ref_frame, 'base_link', rclpy.time.Time())
            rx = robot_tf.transform.translation.x
            ry = robot_tf.transform.translation.y

            angle_to_tag = math.atan2(self.current_tag_pose_target[1] - ry, self.current_tag_pose_target[0] - rx)
            goal_x = self.current_tag_pose_target[0] - STANDOFF_DISTANCE_M * math.cos(angle_to_tag)
            goal_y = self.current_tag_pose_target[1] - STANDOFF_DISTANCE_M * math.sin(angle_to_tag)

            self.navigating_to_standoff = True
            self.send_nav_goal(goal_x, goal_y, angle_to_tag, ref_frame)
        except TransformException as ex:
            self.get_logger().error(f"Robot base_link TF lookup in '{ref_frame}' failed: {ex}")
            self.aligning_camera = True

    def send_nav_goal(self, gx: float, gy: float, yaw: float, frame_id: str):
        if not self.nav_client.wait_for_server(timeout_sec=NAV_TIMEOUT_SEC):
            self.get_logger().warn("Nav2 server unavailable. Proceeding with visual alignment directly.")
            self.aligning_camera = True
            return

        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected approach goal. Starting in-place camera alignment.")
            self.aligning_camera = True
            return

        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        self.navigating_to_standoff = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Arrived at standoff position. Verifying optical alignment...")
            self.aligning_camera = True
        else:
            self.get_logger().error(f"Nav2 approach aborted (status {status}). Cannot reach inspection point!")
            self.resume_exploration()

    def execute_visual_servoing(self, gray_img):
        try:
            detections = self.detector.detect(
                gray_img, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=self.tag_size
            )
        except Exception:
            detections = []
            
        target = next((d for d in detections if d.tag_id == self.current_tag_id), None)

        twist = Twist()
        if target is not None:
            dist = float(np.linalg.norm(target.pose_t))

            # In bypass_nav or standalone mode, don't abort if we're further away during bench testing
            if not self.bypass_nav and dist > MAX_SCAN_DISTANCE_M:
                self.get_logger().warn(f"Tag #{self.current_tag_id} visible but too far ({dist:.2f}m).")
                self.resume_exploration()
                return

            tag_center_x = target.center[0]
            frame_center_x = self.image_width / 2.0
            pixel_error = frame_center_x - tag_center_x

            # Deadband check or bypass check
            if self.bypass_nav or abs(pixel_error) <= IMAGE_CENTER_DEADBAND_PX:
                self.cmd_vel_pub.publish(Twist())  # Stop rotating
                self.update_marker_close_range(target)
                self.aligning_camera = False
                self.inspecting_qr = True
                self.get_logger().info(f"Tag #{self.current_tag_id} centered ({dist:.2f}m). Scanning QR payload...")
                return

            twist.angular.z = ALIGN_ANGULAR_SPEED if pixel_error > 0 else -ALIGN_ANGULAR_SPEED
            self.cmd_vel_pub.publish(twist)
        else:
            if self.bypass_nav:
                # If benchmarking and tag momentarily flickers out, jump straight to QR scanning
                self.aligning_camera = False
                self.inspecting_qr = True
            else:
                twist.angular.z = ALIGN_ANGULAR_SPEED
                self.cmd_vel_pub.publish(twist)

    def update_marker_close_range(self, detection):
        tag_cam_pose = PoseStamped()
        tag_cam_pose.header.frame_id = 'camera_color_optical_frame'
        tag_cam_pose.header.stamp = rclpy.time.Time().to_msg()
        tag_cam_pose.pose.position.x = float(detection.pose_t[0])
        tag_cam_pose.pose.position.y = float(detection.pose_t[1])
        tag_cam_pose.pose.position.z = float(detection.pose_t[2])
        tag_cam_pose.pose.orientation.w = 1.0

        refined_pose, ref_frame = self.transform_pose_with_fallback(tag_cam_pose)
        if refined_pose:
            self.current_tag_pose_stamped = refined_pose
            self.current_tag_pose_target = (refined_pose.pose.position.x, refined_pose.pose.position.y)
            self.publish_markers(self.current_tag_id, refined_pose, "[INSPECTING]", is_solved=False)
            self.get_logger().info(
                f"Locked refined marker in '{ref_frame}' at [{refined_pose.pose.position.x:.2f}, {refined_pose.pose.position.y:.2f}]"
            )

    def inspect_qr_payload(self, cv_image):
        self.qr_search_counter += 1
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # 1. Re-detect the target AprilTag to get its current pixel position
        try:
            detections = self.detector.detect(
                gray, estimate_tag_pose=False
            )
        except Exception:
            detections = []

        target_apriltag = next((d for d in detections if d.tag_id == self.current_tag_id), None)
        qr_results = decode_qr(gray)

        if qr_results:
            selected_qr = None

            # If the AprilTag is in view, pick the QR closest to it
            if target_apriltag is not None:
                tag_cx, tag_cy = target_apriltag.center
                
                def qr_dist_to_tag(qr):
                    rect = qr.rect
                    qr_cx = rect.left + rect.width / 2.0
                    qr_cy = rect.top + rect.height / 2.0
                    return math.hypot(qr_cx - tag_cx, qr_cy - tag_cy)

                selected_qr = min(qr_results, key=qr_dist_to_tag)
            else:
                # Fallback: Pick the QR closest to the optical center of the camera
                img_cx = self.image_width / 2.0
                img_cy = cv_image.shape[0] / 2.0
                def qr_dist_to_center(qr):
                    rect = qr.rect
                    qr_cx = rect.left + rect.width / 2.0
                    qr_cy = rect.top + rect.height / 2.0
                    return math.hypot(qr_cx - img_cx, qr_cy - img_cy)

                selected_qr = min(qr_results, key=qr_dist_to_center)

            payload = selected_qr.data.decode('utf-8')
            self.get_logger().info(f">>> Spatially Matched QR Payload: {payload} <<<")

            # Parse query tag param if present to reconcile ID differences
            final_tag_id = self.current_tag_id
            try:
                parsed = urlparse(payload)
                query_params = parse_qs(parsed.query)
                if 'tag' in query_params:
                    qr_tag_id = int(query_params['tag'][0])
                    if qr_tag_id != self.current_tag_id:
                        self.get_logger().warn(
                            f"Syncing AprilTag #{self.current_tag_id} -> QR payload Tag #{qr_tag_id}"
                        )
                    final_tag_id = qr_tag_id
            except Exception as e:
                self.get_logger().warn(f"Failed to parse query param from QR: {e}")

            # Register both IDs as discovered/tracked so neither re-triggers
            self.discovered_tags.add(self.current_tag_id)
            self.discovered_tags.add(final_tag_id)
            self.current_tag_id = final_tag_id

            self.publish_discovery(final_tag_id, payload, self.current_tag_pose_target)
            self.inspecting_qr = False
            self.waiting_for_puzzle = True
            self.puzzle_wait_start_time = self.get_clock().now()
            self.puzzle_timeout_notified = False
            self.get_logger().info(f"Tag #{self.current_tag_id} captured. Waiting for puzzle solve on web UI...")
            return

        # Bench fallback:
        if self.standalone_test_mode and self.qr_search_counter >= 15:
            fake_url = f"http://localhost:8000/inspect?tag={self.current_tag_id}"
            self.get_logger().warn(f"[Standalone Test] Simulating QR payload decode: '{fake_url}'")
            self.publish_discovery(self.current_tag_id, fake_url, self.current_tag_pose_target)
            self.discovered_tags.add(self.current_tag_id)
            self.inspecting_qr = False
            self.waiting_for_puzzle = True
            return

        if self.qr_search_counter >= INSPECTION_SETTLE_FRAMES:
            self.qr_retry_counter += 1
            if self.qr_retry_counter < MAX_QR_RETRIES:
                self.get_logger().warn(f"Retry {self.qr_retry_counter}/{MAX_QR_RETRIES} for Tag #{self.current_tag_id}")
                self.qr_search_counter = 0
            else:
                self.trigger_help_request()

    def trigger_help_request(self):
        self.awaiting_help = True
        self.inspecting_qr = False
        alert_msg = {
            "status": "HELP_REQUIRED",
            "reason": "QR_DECODE_FAILED",
            "tag_id": self.current_tag_id,
            "x": round(self.current_tag_pose_target[0], 3),
            "y": round(self.current_tag_pose_target[1], 3)
        }
        self.help_pub.publish(String(data=json.dumps(alert_msg)))
        self.get_logger().error(f"🚨 [HELP REQUIRED] Tag #{self.current_tag_id} failed after {MAX_QR_RETRIES} attempts.")

    def publish_discovery(self, tag_id: int, payload: str, tag_pose):
        msg_payload = {
            "tag_id": tag_id,
            "payload": payload,
            "x": round(tag_pose[0], 3),
            "y": round(tag_pose[1], 3)
        }
        self.artifact_pub.publish(String(data=json.dumps(msg_payload)))

    def resume_exploration(self):
        self.active_inspection = False
        self.navigating_to_standoff = False
        self.aligning_camera = False
        self.inspecting_qr = False
        self.waiting_for_puzzle = False
        self.awaiting_help = False
        self.current_tag_id = None
        self.qr_search_counter = 0
        self.qr_retry_counter = 0
        self.tag_pose_buffers.clear()

        self.request_nav_pub.publish(String(data='release'))
        self.explore_ctrl_pub.publish(String(data='start'))
        self.get_logger().info("Inspection cleared. Nav2 lock released back to Explorer.")

    def publish_markers(self, tag_id: int, pose_stamped: PoseStamped, label_text: str, is_solved: bool = False):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        ref_frame = pose_stamped.header.frame_id

        marker = Marker()
        marker.header.frame_id = ref_frame
        marker.header.stamp = now
        marker.ns = 'artifacts'
        marker.id = tag_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        if is_solved:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.2, 0.95
        else:
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.8, 0.0, 0.95
        marker_array.markers.append(marker)

        text_marker = Marker()
        text_marker.header.frame_id = ref_frame
        text_marker.header.stamp = now
        text_marker.ns = 'artifact_labels'
        text_marker.id = tag_id + 10000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = pose_stamped.pose.position.x
        text_marker.pose.position.y = pose_stamped.pose.position.y
        text_marker.pose.position.z = pose_stamped.pose.position.z + 0.20
        text_marker.scale.z = 0.08
        text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 1.0, 1.0, 1.0
        text_marker.text = f"Tag #{tag_id} {label_text}"
        marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ArtifactInspectorNode()
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