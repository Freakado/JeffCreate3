#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Vector3, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from action_msgs.msg import GoalStatus
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformException

from irobot_create_msgs.action import Undock, Dock
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector

# --- Timers & Frequencies ---
LOOP_RATE_HZ                  = 2.0
LOOP_PERIOD_SEC               = 1.0 / LOOP_RATE_HZ
TF_THROTTLE_SEC               = 4.0
NAV_SERVER_THROTTLE_SEC       = 3.0
NAV_SERVER_TIMEOUT_SEC        = 1.0

# --- Frontier Raycast Parameters ---
RAY_STEP_RESOLUTION_M        = 0.10
MIN_CLEAR_PATH_M              = 1.10   # Disqualifies tight furniture gaps and short alcoves
GOAL_STANDOFF_FROM_WALL_M     = 0.55   # Standoff to keep target outside the wall inflation zone
VISITED_SPACING_M             = 1.30   # Minimum spacing to prevent clustering
MOMENTUM_WEIGHT               = 2.0    # Forward persistence bias through doors
UNKNOWN_CELL_WEIGHT           = 3.0    # Reward for undiscovered map space

# --- Coordinate Validation & Costmap Thresholds ---
BLACKLIST_PROXIMITY_RADIUS_M  = 0.50   # Expanded to keep clear of problematic zones
SLAM_WALL_OCCUPANCY_THRESHOLD = 50
COSTMAP_LETHAL_THRESHOLD      = 90

# --- Recovery Spin Parameters ---
EMPTY_STACK_CYCLE_THRESHOLD   = 3
RECOVERY_SPIN_TIMER_PERIOD_S  = 0.1
RECOVERY_SPIN_DURATION_SEC    = 5.0
RECOVERY_SPIN_ANGULAR_VEL     = 0.45
MAX_CONSECUTIVE_SPINS_BEFORE_RESET = 3

# --- Marker Visualization ---
MARKER_SCALE_XYZ              = (0.15, 0.15, 0.15)
MARKER_COLOR_RGBA             = (0.0, 1.0, 0.2, 1.0)
MARKER_Z_OFFSET_M             = 0.10


class JeffCreate3Explorer(Node):
    def __init__(self):
        super().__init__('explorer_node')

        # Tunable Parameters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('step_distance', 1.3)
        self.declare_parameter('max_distance', 2.5)
        self.declare_parameter('obstacle_clearance', 0.6)
        
        # Kill / Discard Parameters
        self.declare_parameter('goal_timeout_sec', 18.0)       # Kill goal if not reached within 18s
        self.declare_parameter('max_hazard_strikes', 2)        # Kill goal after 2 bumper/hazard events
        
        # Resume arbitration: 'explicit' or 'auto'
        self.declare_parameter('manual_resume_mode', 'explicit')
        self.declare_parameter('auto_resume_delay_sec', 4.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.step_dist = self.get_parameter('step_distance').value
        self.max_dist = self.get_parameter('max_distance').value
        self.clearance = self.get_parameter('obstacle_clearance').value
        self.goal_timeout_sec = self.get_parameter('goal_timeout_sec').value
        self.max_hazard_strikes = self.get_parameter('max_hazard_strikes').value
        self.manual_resume_mode = self.get_parameter('manual_resume_mode').value.lower()
        self.auto_resume_delay = self.get_parameter('auto_resume_delay_sec').value

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State Tracking
        self.is_active = False
        self.is_navigating = False
        self.is_recovering = False
        self.current_goal_handle = None
        self.current_goal_data = None
        self.current_goal_start_time = None
        self.current_hazard_strikes = 0

        self.current_pos = [0.0, 0.0, 0.0]
        self.has_pose = False
        self.visited_goals = []
        self.blacklisted_goals = []
        self.latest_map = None
        self.latest_costmap = None
        self.empty_stack_cycles = 0
        self.consecutive_spins = 0

        # Nav2 Arbitration State ('explorer', 'inspector', 'manual')
        self.nav_owner = "explorer"
        self.auto_resume_timer = None

        # Docking tracking
        self.is_docked = False
        self.dock_pose_map = None
        self.undocking_in_progress = False

        # QoS Profiles
        sensor_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        latched_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        operator_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'stack_points', 5)
        self.active_nav_pub = self.create_publisher(String, 'active_nav2', latched_qos)

        # Subscriptions
        self.start_sub = self.create_subscription(String, 'start_explore', self.start_callback, latched_qos)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, sensor_qos)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, latched_qos)
        self.costmap_sub = self.create_subscription(OccupancyGrid, 'global_costmap/costmap', self.costmap_callback, latched_qos)
        self.hazard_sub = self.create_subscription(HazardDetectionVector, 'hazard_detection', self.hazard_callback, sensor_qos)

        # Arbitration Subscriptions
        self.request_nav_sub = self.create_subscription(String, 'request_nav2', self.nav_request_callback, operator_qos)
        self.rviz_goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.rviz_goal_callback, operator_qos)
        self.clear_help_sub = self.create_subscription(String, 'clear_help', self.clear_help_callback, operator_qos)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Create 3 Docking Clients
        self.undock_client = ActionClient(self, Undock, 'undock')
        self.dock_client = ActionClient(self, Dock, 'dock')
        self.dock_status_sub = self.create_subscription(DockStatus, 'dock_status', self.dock_status_callback, 10)

        self.clear_local_costmap = self.create_client(ClearEntireCostmap, 'local_costmap/clear_entirely_local_costmap')
        self.clear_global_costmap = self.create_client(ClearEntireCostmap, 'global_costmap/clear_entirely_global_costmap')

        # Publish initial ownership state
        self.active_nav_pub.publish(String(data='explorer'))

        # Periodic State Machine
        self.timer = self.create_timer(LOOP_PERIOD_SEC, self.state_machine_loop)
        self.get_logger().info(
            f'Explorer active with Watchdogs: timeout={self.goal_timeout_sec}s, max_strikes={self.max_hazard_strikes}'
        )

    # --- Hazard & Timeout Watchdogs ---
    def hazard_callback(self, msg: HazardDetectionVector):
        """Immediately halts the robot and cancels the goal upon physical bumper/cliff hit."""
        if not self.is_navigating or self.nav_owner != 'explorer':
            return

        # Trigger on BUMP (1) or CLIFF (2)
        physical_hits = [h for h in msg.detections if h.type in (1, 2)]
        if physical_hits:
            self.get_logger().error("💥 Bumper/Hazard contact triggered! Halting robot and killing impossible path.")
            # 1. Zero velocity stop immediately
            self.cmd_vel_pub.publish(Twist())
            # 2. Cancel Nav2 goal and blacklist location
            self.kill_current_goal(reason='PHYSICAL_BUMP')

    def check_goal_timeout(self):
        """Kills goals that linger longer than goal_timeout_sec."""
        if not self.is_navigating or self.current_goal_start_time is None:
            return

        elapsed = (self.get_clock().now() - self.current_goal_start_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_sec:
            self.get_logger().error(
                f'Goal timeout! Took {elapsed:.1f}s > {self.goal_timeout_sec}s. Killing goal and moving on.'
            )
            self.kill_current_goal(reason='TIMEOUT')

    def kill_current_goal(self, reason: str = 'WATCHDOG'):
        if self.current_goal_data is not None:
            gx, gy = self.current_goal_data[0], self.current_goal_data[1]
            self.blacklisted_goals.append((gx, gy))
            self.get_logger().info(f'Blacklisted unreachable target [{gx:.2f}, {gy:.2f}] (Reason: {reason})')

        self.cancel_active_navigation()
        self.current_goal_data = None
        self.current_goal_start_time = None
        self.current_hazard_strikes = 0

    # --- Arbitration Callbacks ---
    def nav_request_callback(self, msg: String):
        requester = msg.data.strip().lower()
        if requester == 'inspector':
            self.get_logger().warn('Yielding Nav2 control to Inspector.')
            self.nav_owner = 'inspector'
            self.cancel_active_navigation()
            self.is_active = False
            self.active_nav_pub.publish(String(data='inspector'))

        elif requester in ('explorer', 'release'):
            self.get_logger().info('Nav2 ownership restored to Explorer.')
            self.nav_owner = 'explorer'
            self.is_active = True
            self.active_nav_pub.publish(String(data='explorer'))

    def rviz_goal_callback(self, msg: PoseStamped):
        self.get_logger().warn(
            f'Manual 2D Goal received on /goal_pose [{msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}]. '
            'Halting explorer autonomous goals.'
        )
        self.nav_owner = 'manual'
        self.cancel_active_navigation()
        self.is_active = False
        self.active_nav_pub.publish(String(data='manual'))

        if self.manual_resume_mode == 'auto':
            if self.auto_resume_timer is not None:
                self.auto_resume_timer.cancel()
            self.auto_resume_timer = self.create_timer(1.0, self.check_manual_goal_completion)

    def check_manual_goal_completion(self):
        if self.nav_owner != 'manual':
            if self.auto_resume_timer:
                self.auto_resume_timer.cancel()
            return

        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            dist_moved = math.hypot(t.transform.translation.x - self.current_pos[0],
                                    t.transform.translation.y - self.current_pos[1])
            if dist_moved < 0.05:
                self.get_logger().info(
                    f'Manual goal settled. Resuming explorer in {self.auto_resume_delay:.1f}s...'
                )
                if self.auto_resume_timer:
                    self.auto_resume_timer.cancel()
                self.auto_resume_timer = self.create_timer(self.auto_resume_delay, self.auto_resume_timeout)
        except Exception:
            pass

    def auto_resume_timeout(self):
        if self.auto_resume_timer:
            self.auto_resume_timer.cancel()
            self.auto_resume_timer = None
        if self.nav_owner == 'manual':
            self.get_logger().info('Auto-resume delay expired. Restoring autonomous exploration.')
            self.nav_owner = 'explorer'
            self.is_active = True
            self.active_nav_pub.publish(String(data='explorer'))

    def clear_help_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd in ('resume', 'retry', 'skip') and self.nav_owner == 'manual':
            self.get_logger().info(f'Operator issued /{cmd}. Restoring autonomous exploration.')
            self.nav_owner = 'explorer'
            self.is_active = True
            self.active_nav_pub.publish(String(data='explorer'))

    def cancel_active_navigation(self):
        self.is_navigating = False
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:
                pass
            self.current_goal_handle = None

    def dock_status_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def costmap_callback(self, msg: OccupancyGrid):
        self.latest_costmap = msg

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def start_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'start':
            if self.nav_owner == 'inspector':
                self.get_logger().warn('Ignoring start command: Inspector currently holds Nav2 lock.')
                return

            if self.is_docked and not self.undocking_in_progress:
                self.get_logger().info('Robot is docked. Initiating undock sequence...')
                self.trigger_undock_sequence()
            else:
                self.nav_owner = 'explorer'
                self.is_active = True
                self.active_nav_pub.publish(String(data='explorer'))
                self.get_logger().info('Exploration activated.')

        elif cmd == 'stop':
            self.is_active = False
            self.cancel_active_navigation()
            self.is_recovering = False
            self.get_logger().info('Exploration paused.')

        elif cmd in ('return', 'dock'):
            self.handle_return_and_dock()

    def update_robot_pose(self) -> bool:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            self.current_pos[0] = t.transform.translation.x
            self.current_pos[1] = t.transform.translation.y
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.current_pos[2] = yaw
            self.has_pose = True
            return True
        except TransformException as ex:
            self.get_logger().warn(
                f'Waiting for transform ({self.map_frame} -> {self.base_frame}): {ex}',
                throttle_duration_sec=TF_THROTTLE_SEC
            )
            return False

    def scan_callback(self, msg: LaserScan):
        pass  # Map raycasting handles obstacle clearance

    def is_coordinate_valid(self, gx: float, gy: float) -> bool:
        for bx, by in self.blacklisted_goals:
            if math.hypot(gx - bx, gy - by) < BLACKLIST_PROXIMITY_RADIUS_M:
                return False

        if self.latest_map is not None:
            info = self.latest_map.info
            mx = math.floor((gx - info.origin.position.x) / info.resolution)
            my = math.floor((gy - info.origin.position.y) / info.resolution)

            MARGIN = 3
            if mx < MARGIN or mx >= (info.width - MARGIN) or my < MARGIN or my >= (info.height - MARGIN):
                return False

            idx = my * info.width + mx
            if 0 <= idx < len(self.latest_map.data):
                val = self.latest_map.data[idx]
                if val > SLAM_WALL_OCCUPANCY_THRESHOLD:
                    return False

        if self.latest_costmap is not None:
            c_info = self.latest_costmap.info
            cx = math.floor((gx - c_info.origin.position.x) / c_info.resolution)
            cy = math.floor((gy - c_info.origin.position.y) / c_info.resolution)

            C_MARGIN = 3
            if cx < C_MARGIN or cx >= (c_info.width - C_MARGIN) or cy < C_MARGIN or cy >= (c_info.height - C_MARGIN):
                return False

            c_idx = cy * c_info.width + cx
            if 0 <= c_idx < len(self.latest_costmap.data):
                cost = self.latest_costmap.data[c_idx]
                if cost >= COSTMAP_LETHAL_THRESHOLD:
                    return False

        return True

    def evaluate_best_frontier_goal(self):
        if self.latest_map is None:
            return None

        map_info = self.latest_map.info
        res = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        map_w = map_info.width
        map_h = map_info.height
        map_data = np.array(self.latest_map.data).reshape((map_h, map_w))

        rx, ry, ryaw = self.current_pos
        best_score = -float('inf')
        best_goal = None

        angles = np.linspace(-math.pi, math.pi, 16, endpoint=False)

        for heading in angles:
            angle_diff = abs(math.atan2(math.sin(heading - ryaw), math.cos(heading - ryaw)))
            momentum_score = math.cos(angle_diff) * MOMENTUM_WEIGHT

            unknown_cells = 0
            max_clear_travel = 0.0

            steps = int(self.max_dist / RAY_STEP_RESOLUTION_M)
            for step in range(1, steps + 1):
                d = step * RAY_STEP_RESOLUTION_M
                cx = rx + d * math.cos(heading)
                cy = ry + d * math.sin(heading)

                mx = int((cx - origin_x) / res)
                my = int((cy - origin_y) / res)

                if mx < 0 or mx >= map_w or my < 0 or my >= map_h:
                    unknown_cells += 2
                    continue

                cell_val = map_data[my, mx]
                if cell_val >= SLAM_WALL_OCCUPANCY_THRESHOLD:
                    break
                elif cell_val == -1:
                    unknown_cells += 1

                max_clear_travel = d

            if max_clear_travel < MIN_CLEAR_PATH_M:
                continue

            target_dist = min(max_clear_travel - GOAL_STANDOFF_FROM_WALL_M, self.step_dist)
            if target_dist < 0.60:
                continue

            gx = rx + target_dist * math.cos(heading)
            gy = ry + target_dist * math.sin(heading)

            if not self.is_coordinate_valid(gx, gy):
                continue

            visited_penalty = 0.0
            for vx, vy, _, _, _, _ in self.visited_goals[-12:]:
                dist_prev = math.hypot(gx - vx, gy - vy)
                if dist_prev < VISITED_SPACING_M:
                    visited_penalty += (VISITED_SPACING_M - dist_prev) * 4.5

            score = (unknown_cells * UNKNOWN_CELL_WEIGHT) + momentum_score - visited_penalty

            if score > best_score:
                best_score = score
                q = quaternion_from_euler(0.0, 0.0, heading)
                best_goal = [gx, gy, q[0], q[1], q[2], q[3]]

        return best_goal

    def state_machine_loop(self):
        if self.nav_owner != "explorer":
            return

        # Check ongoing goal health
        if self.is_navigating:
            self.check_goal_timeout()
            return

        if not self.is_active or self.is_recovering:
            return

        if not self.update_robot_pose():
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn('Waiting for Nav2 action server...', throttle_duration_sec=NAV_SERVER_THROTTLE_SEC)
            return

        target_pose = self.evaluate_best_frontier_goal()

        if target_pose is None:
            self.empty_stack_cycles += 1
            if self.empty_stack_cycles >= EMPTY_STACK_CYCLE_THRESHOLD:
                self.get_logger().info('No viable open frontiers found. Initiating recovery spin...')
                self.empty_stack_cycles = 0
                self.trigger_recovery_spin()
            return

        self.empty_stack_cycles = 0
        self.visited_goals.append(target_pose)
        self.publish_markers()
        self.send_nav_goal(target_pose)

    def send_nav_goal(self, goal_data):
        if not self.nav_client.wait_for_server(timeout_sec=NAV_SERVER_TIMEOUT_SEC):
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_data[0]
        goal_msg.pose.pose.position.y = goal_data[1]
        goal_msg.pose.pose.orientation.x = goal_data[2]
        goal_msg.pose.pose.orientation.y = goal_data[3]
        goal_msg.pose.pose.orientation.z = goal_data[4]
        goal_msg.pose.pose.orientation.w = goal_data[5]

        self.is_navigating = True
        self.current_goal_data = goal_data
        self.current_goal_start_time = self.get_clock().now()
        self.current_hazard_strikes = 0

        self.get_logger().info(f'Navigating to frontier: [{goal_data[0]:.2f}, {goal_data[1]:.2f}]')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda f: self.goal_response_callback(f, goal_data))

    def goal_response_callback(self, future, goal_data):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal.')
            self.blacklisted_goals.append((goal_data[0], goal_data[1]))
            self.is_navigating = False
            self.current_goal_data = None
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda rf: self.goal_result_callback(rf, goal_data))

    def goal_result_callback(self, future, goal_data):
        status = future.result().status
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_goal_data = None
        self.current_goal_start_time = None
        self.current_hazard_strikes = 0

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached target [{goal_data[0]:.2f}, {goal_data[1]:.2f}]')
            self.consecutive_spins = 0
        elif status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Navigation aborted (status {status}). Blacklisting.')
            self.blacklisted_goals.append((goal_data[0], goal_data[1]))

    def trigger_recovery_spin(self):
        self.is_recovering = True
        self.spin_start_time = self.get_clock().now()
        self.spin_timer = self.create_timer(RECOVERY_SPIN_TIMER_PERIOD_S, self.execute_recovery_spin)

    def execute_recovery_spin(self):
        if self.consecutive_spins >= MAX_CONSECUTIVE_SPINS_BEFORE_RESET:
            self.reset_navigation_state()
        elapsed = (self.get_clock().now() - self.spin_start_time).nanoseconds / 1e9

        if elapsed < RECOVERY_SPIN_DURATION_SEC:
            twist = Twist()
            twist.angular.z = RECOVERY_SPIN_ANGULAR_VEL
            self.cmd_vel_pub.publish(twist)
        else:
            self.cmd_vel_pub.publish(Twist())
            self.spin_timer.cancel()
            self.destroy_timer(self.spin_timer)
            self.is_recovering = False
            self.consecutive_spins += 1
            # Clear local costmap residual scan trail right after the spin
            req = ClearEntireCostmap.Request()
            if self.clear_local_costmap.service_is_ready():
                self.clear_local_costmap.call_async(req)

    def publish_markers(self):
        marker_array = MarkerArray()
        for idx, pt in enumerate(self.visited_goals[-25:]):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.pose.position.z = MARKER_Z_OFFSET_M
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=MARKER_SCALE_XYZ[0], y=MARKER_SCALE_XYZ[1], z=MARKER_SCALE_XYZ[2])
            marker.color = ColorRGBA(r=MARKER_COLOR_RGBA[0], g=MARKER_COLOR_RGBA[1], b=MARKER_COLOR_RGBA[2], a=MARKER_COLOR_RGBA[3])
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def reset_navigation_state(self):
        req = ClearEntireCostmap.Request()
        if self.clear_local_costmap.service_is_ready():
            self.clear_local_costmap.call_async(req)
        if self.clear_global_costmap.service_is_ready():
            self.clear_global_costmap.call_async(req)
        self.blacklisted_goals.clear()
        self.consecutive_spins = 0

    def trigger_undock_sequence(self):
        if not self.undock_client.wait_for_server(timeout_sec=3.0):
            self.is_active = True
            return
        self.undocking_in_progress = True
        goal_msg = Undock.Goal()
        send_future = self.undock_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.undocking_in_progress = False
            self.is_active = True
            return
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.undock_completed_callback)

    def undock_completed_callback(self, future):
        self.undocking_in_progress = False
        if self.has_pose:
            self.dock_pose_map = (self.current_pos[0], self.current_pos[1], self.current_pos[2] + math.pi)
        self.is_active = True

    def handle_return_and_dock(self):
        self.is_active = False
        self.is_recovering = False
        self.cancel_active_navigation()
        if self.is_docked or self.dock_pose_map is None:
            return
        gx, gy, yaw = self.dock_pose_map
        self.send_return_goal(gx, gy, yaw)

    def send_return_goal(self, gx: float, gy: float, yaw: float):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.map_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = gx
        goal_msg.pose.pose.position.y = gy
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.is_navigating = True
        send_future = self.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.return_goal_response_callback)

    def return_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_navigating = False
            return
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.return_goal_result_callback)

    def return_goal_result_callback(self, future):
        self.is_navigating = False
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.trigger_dock_sequence()

    def trigger_dock_sequence(self):
        if not self.dock_client.wait_for_server(timeout_sec=3.0):
            return
        goal_msg = Dock.Goal()
        send_future = self.dock_client.send_goal_async(goal_msg)
        send_future.add_done_callback(lambda f: f.result().get_result_async())


def main(args=None):
    rclpy.init(args=args)
    node = JeffCreate3Explorer()
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