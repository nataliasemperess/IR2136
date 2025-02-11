import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import time 

class BatteryGpsNode(Node):

    def __init__(self):

        super().__init__('battery_gps_node')
        
        # Initialize MAVLink connection
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info('Connected to SITL.')
        
        # Battery status publisher
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)
        
        # GPS position subscriber
        self.create_subscription(Point, 'target_gps', self.target_gps_callback, 10)
        
        # Timer to publish battery status
        self.timer = self.create_timer(5.0, self.publish_battery_status)
        
    def publish_battery_status(self):
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            remaining = msg.battery_remaining  # Remaining battery percentage
            battery_msg = Float32()
            battery_msg.data = remaining
            self.battery_pub.publish(battery_msg)
            self.get_logger().info(f'Battery: {remaining}%')
    
    def target_gps_callback(self, msg):
        lat, lon, alt = msg.x, msg.y, msg.z
        self.get_logger().info(f'Received target GPS position: Lat={lat}, Lon={lon}, Alt={alt}m')
        
        # Send target GPS to drone
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,  # Ignore velocity and acceleration
            int(lat * 1e7),  # Latitude as integer
            int(lon * 1e7),  # Longitude as integer
            alt,  # Desired altitude
            0, 0, 0,  # Velocities
            0, 0, 0,  # Accelerations
            0, 0  # Yaw and Yaw rate
        )
        self.get_logger().info(f"Moving drone to Lat={lat}, Lon={lon}, Alt={alt}m")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryGpsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
