#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature

class NucTempMonitorNode(Node):
    """
    A ROS2 node that reads the Intel NUC's CPU temperature and publishes it.
    """
    def __init__(self):
        # Updated node name to reflect the NUC hardware platform
        super().__init__('nuc_temp_monitor_node')
        self.get_logger().info('Initializing NUC G5 Temperature Monitor Node...')

        # Core Linux thermal zone file path (Valid for both NUC and Raspberry Pi cores)
        self.temp_file_path = '/sys/class/thermal/thermal_zone0/temp'

        # --- ROS2 Publisher Setup ---
        # Standard sensor_msgs/Temperature topic format
        self.publisher_ = self.create_publisher(Temperature, 'nuc_cpu_temperature', 10)

        # --- Timer Setup ---
        # Executes the sampling cycle once every second
        self.timer = self.create_timer(1.0, self.read_and_publish_temp)

        self.get_logger().info('Node started. Publishing NUC CPU temperature every second.')

    def read_and_publish_temp(self):
        """
        Reads the temperature from the system file and publishes it.
        """
        try:
            with open(self.temp_file_path, 'r') as temp_file:
                temp_str = temp_file.read()
                # The hardware registry value is in milli-degrees C, so we scale by 1000.0
                temp_celsius = float(temp_str) / 1000.0

                # Assemble the standard ROS2 Temperature Message structure
                msg = Temperature()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'nuc_cpu_thermal_zone'

                # Set the payload value
                msg.temperature = temp_celsius
                msg.variance = 0.0 # Unmeasured variance defaults to 0.0

                # Publish onto the local ROS2 domain
                self.publisher_.publish(msg)

                # Optional debug statement (Uncomment if you want to inspect via terminal)
                # self.get_logger().info(f'Published NUC CPU Temp: {temp_celsius:.2f} °C')

        except FileNotFoundError:
            self.get_logger().error(f"Temperature file registry target missing at: {self.temp_file_path}")
        except ValueError:
            self.get_logger().error("Could not parse system registry entry string into a numeric float.")
        except Exception as e:
            self.get_logger().error(f"An unexpected runtime exception occurred: {e}")


def main(args=None):
    rclpy.init(args=args)
    # FIXED: Instantiates the exact matching class name defined above
    temp_monitor_node = NucTempMonitorNode()
    try:
        rclpy.spin(temp_monitor_node)
    except KeyboardInterrupt:
        pass
    finally:
        temp_monitor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
