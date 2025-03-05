import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from pymavlink import mavutil
import sys
import time

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        # Inicializar la conexión MAVLink
        self.connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.connection.wait_heartbeat()
        self.get_logger().info('Conectado al SITL.')

        # Publicador de la posición GPS objetivo
        self.target_gps_pub = self.create_publisher(Point, 'target_gps', 10)
        
        # Suscriptor para el estado de la batería
        self.create_subscription(Float32, 'battery_status', self.battery_callback, 10)
        
        # Suscriptor para recibir la posición GPS objetivo
        self.create_subscription(Point, 'target_gps', self.target_gps_callback, 10)

        # Verificar argumentos
        if len(sys.argv) != 4:
            self.get_logger().error("Uso: mission_control_node.py <latitud> <longitud> <altitud>")
            sys.exit(1)

        # Extraer coordenadas
        self.target_lat = float(sys.argv[1])
        self.target_lon = float(sys.argv[2])
        self.target_alt = float(sys.argv[3])

        # Configurar el dron
        self.set_mode('GUIDED')
        self.arm_vehicle()
        self.takeoff()
        
        # Esperar antes de enviar destino
        time.sleep(10)
        self.publish_target_gps(self.target_lat, self.target_lon, self.target_alt)
    
    def arm_vehicle(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)
        self.get_logger().info("Drone armed.")
    
    def takeoff(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.target_alt)
        self.get_logger().info("Taking off.")
    
    def set_mode(self, mode):
        mode_mapping = {
            'GUIDED': 4,  # GUIDED mode en ArduPilot
            'LAND': 9  # LAND mode en ArduPilot
        }
        if mode in mode_mapping:
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_mapping[mode])
            self.get_logger().info(f"Mode changed to {mode}")
    
    def publish_target_gps(self, lat, lon, alt):
        target_msg = Point()
        target_msg.x = lat
        target_msg.y = lon
        target_msg.z = alt
        self.target_gps_pub.publish(target_msg)
        self.get_logger().info(f"Target GPS position published: {lat}, {lon}, {alt}")
    
    def target_gps_callback(self, msg):
        """Callback para recibir la posición GPS objetivo."""
        lat, lon, alt = msg.x, msg.y, msg.z
        self.get_logger().info(f"Recibida posición GPS objetivo: Lat={lat}, Lon={lon}, Alt={alt}m")

        # Enviar la posición GPS al dron
        self.send_gps_to_drone(lat, lon, alt)

    def battery_callback(self, msg):
        if msg.data < 20.0:
            self.get_logger().warn("Low battery! Initiating landing...")
            self.land_drone()
    
    def send_gps_to_drone(self, lat, lon, alt):
        self.connection.mav.set_position_target_global_int_send(
            0,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.get_logger().info(f"Enviando al dron: Lat={lat}, Lon={lon}, Alt={alt}m")
    
    def land_drone(self):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0)
        self.get_logger().info("Landing command sent.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()