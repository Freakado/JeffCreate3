#!/usr/bin/env python3

import os
import signal
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class RobotManagerNode(Node):
    def __init__(self):
        super().__init__('robot_supervisor_node')

        # Track active subprocesses
        self.processes = {
            'explore': None,
            'inspect': None
        }

        # Topic Subscriber for generic command triggers
        self.cmd_sub = self.create_subscription(
            String,
            'system_command',
            self.system_command_callback,
            10
        )

        # Dedicated ROS 2 Services for clean request/response calls
        self.srv_start_explore = self.create_service(Trigger, 'start_explore_node', self.start_explore_cb)
        self.srv_stop_explore = self.create_service(Trigger, 'stop_explore_node', self.stop_explore_cb)
        self.srv_start_inspect = self.create_service(Trigger, 'start_inspect_node', self.start_inspect_cb)
        self.srv_stop_inspect = self.create_service(Trigger, 'stop_inspect_node', self.stop_inspect_cb)
        self.srv_reboot_nuc = self.create_service(Trigger, 'reboot_nuc', self.reboot_nuc_cb)
        self.srv_reboot_create3 = self.create_service(Trigger, 'reboot_create3', self.reboot_create3_cb)

        self.get_logger().info('Robot Supervisor ready for remote system commands.')

    # --- Subprocess Lifecycle Helpers ---
    def launch_process(self, key: str, cmd: list) -> tuple[bool, str]:
        if self.processes[key] is not None and self.processes[key].poll() is None:
            return False, f"{key} is already running."

        try:
            # start_new_session ensures the whole process group can be killed cleanly later
            self.processes[key] = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f"Launched {key} (PID: {self.processes[key].pid})")
            return True, f"{key} launched successfully."
        except Exception as e:
            return False, f"Failed to launch {key}: {str(e)}"

    def terminate_process(self, key: str) -> tuple[bool, str]:
        proc = self.processes.get(key)
        if proc is None or proc.poll() is not None:
            return False, f"{key} is not currently running."

        try:
            # Terminate the process group (SIGINT for clean ROS shutdown)
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
            self.processes[key] = None
            self.get_logger().info(f"Terminated {key}.")
            return True, f"{key} stopped."
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            self.processes[key] = None
            return True, f"{key} killed forcefully."
        except Exception as e:
            return False, f"Error stopping {key}: {str(e)}"

    # --- Service Callbacks ---
    def start_explore_cb(self, request, response):
        success, message = self.launch_process(
            'explore',
            ['ros2', 'launch', 'jeff_create3_robot', 'explore.launch.py']
        )
        response.success = success
        response.message = message
        return response

    def stop_explore_cb(self, request, response):
        success, message = self.terminate_process('explore')
        response.success = success
        response.message = message
        return response

    def start_inspect_cb(self, request, response):
        success, message = self.launch_process(
            'inspect',
            ['ros2', 'launch', 'jeff_create3_robot', 'inspect.launch.py']
        )
        response.success = success
        response.message = message
        return response

    def stop_inspect_cb(self, request, response):
        success, message = self.terminate_process('inspect')
        response.success = success
        response.message = message
        return response

    def reboot_nuc_cb(self, request, response):
        self.get_logger().warn("Reboot command received for NUC!")
        response.success = True
        response.message = "NUC reboot sequence initiated."
        # Non-blocking delayed reboot
        subprocess.Popen(['sudo', 'reboot'])
        return response

    def reboot_create3_cb(self, request, response):
        self.get_logger().warn("Triggering Create 3 base restart...")
        # Replace with your specific Create 3 reset method (e.g. irobot_create_msgs action or service)
        # Here we invoke the built-in system reset service:
        cmd = ['ros2', 'service', 'call', '/JeffCreate3/system_reset', 'irobot_create_msgs/srv/ResetPose', '{}']
        subprocess.Popen(cmd)
        response.success = True
        response.message = "Create3 reset service invoked."
        return response

    # --- Topic Interface Callback ---
    def system_command_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f"System command received: '{cmd}'")

        if cmd == "launch_explore":
            self.launch_process('explore', ['ros2', 'launch', 'jeff_create3_robot', 'explore.launch.py'])
        elif cmd == "stop_explore":
            self.terminate_process('explore')
        elif cmd == "launch_inspect":
            self.launch_process('inspect', ['ros2', 'launch', 'jeff_create3_robot', 'inspect.launch.py'])
        elif cmd == "stop_inspect":
            self.terminate_process('inspect')
        elif cmd == "reboot_nuc":
            subprocess.Popen(['sudo', 'reboot'])
        elif cmd == "reboot_create3":
            # curl -X POST http://192.168.186.2/api/reboot
            subprocess.Popen(['curl', '-X', 'POST', 'http://192.168.186.2/api/reboot'])
        else:
            self.get_logger().warn(f"Unknown system command: '{cmd}'")

    def destroy_node(self):
        # Clean up spawned processes if supervisor shuts down
        for key in list(self.processes.keys()):
            self.terminate_process(key)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotManagerNode()
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