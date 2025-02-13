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

        # Publicador de la batería
        self.battery_pub = self.create_publisher(Float32, 'battery_status', 10)

        # Publicador de la posición GPS objetivo
        self.target_gps_pub = self.create_publisher(Point, 'target_gps', 10)

        # Suscriptor para la posición GPS objetivo
        self.create_subscription(Point, 'target_gps', self.target_gps_callback, 10)

        # Timer para publicar el estado de la batería cada 5 segundos
        self.timer = self.create_timer(5.0, self.publish_battery_status)

        # Verificar que se pasen correctamente los argumentos
        if len(sys.argv) != 4:
            self.get_logger().error("Uso: mission_control_node.py <latitud> <longitud> <altitud>")
            sys.exit(1)

        # Extraer latitud, longitud y altitud de los argumentos
        self.target_lat = float(sys.argv[1])
        self.target_lon = float(sys.argv[2])
        self.target_alt = float(sys.argv[3])

        # Publicar la posición GPS objetivo
        self.publish_target_gps(self.target_lat, self.target_lon, self.target_alt)

    def arm_vehicle(self):
        """Método para armar el dron."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmación
            1,  # Arm
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Drone armed.")

    def takeoff(self):
        """Método para hacer despegar el dron."""
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmación
            1,  # Arm
            0, 0, 0, 0, 0, 0, self.target_alt  # Altitud deseada
        )
        self.get_logger().info("Taking off")

    def set_mode(self, mode):
        """Método para cambiar el modo del dron."""
        mode_dict = {
            'GUIDED': mavutil.mavlink.MAV_MODE_GUIDED_ARMED,
            'STABILIZE': mavutil.mavlink.MAV_MODE_STABILIZE_ARMED,
            'MANUAL': mavutil.mavlink.MAV_MODE_MANUAL_ARMED,
        }

        mode_value = mode_dict.get(mode, mavutil.mavlink.MAV_MODE_GUIDED_ARMED)
        self.connection.mav.set_mode(self.connection.target_system, mode_value)
        self.get_logger().info(f"Mode changed to {mode}")

    def publish_battery_status(self):
        """Publica el estado de la batería en el tema 'battery_status'."""
        msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
        if msg:
            remaining = msg.battery_remaining  # Porcentaje de batería restante
            battery_msg = Float32()
            battery_msg.data = float(remaining)
            self.battery_pub.publish(battery_msg)
            self.get_logger().info(f'Estado de la batería: {remaining}%')

            # Si la batería está por debajo del 20%, iniciar aterrizaje
            if remaining < 20:
                self.get_logger().info("Batería baja! Iniciando aterrizaje.")
                self.land_drone()

    def target_gps_callback(self, msg):
        """Callback para recibir la posición GPS objetivo."""
        lat, lon, alt = msg.x, msg.y, msg.z
        self.get_logger().info(f"Recibida posición GPS objetivo: Lat={lat}, Lon={lon}, Alt={alt}m")

        # Enviar la posición GPS al dron
        self.send_gps_to_drone(lat, lon, alt)

    def publish_target_gps(self, lat, lon, alt):
        """Publica la posición GPS objetivo al tema 'target_gps'."""
        target_gps_msg = Point()
        target_gps_msg.x = lat
        target_gps_msg.y = lon
        target_gps_msg.z = alt
        self.get_logger().info(f"Publicando posición GPS objetivo: Lat={lat}, Lon={lon}, Alt={alt}")
        self.get_logger().info("Posición GPS publicada.")
        self.target_gps_pub.publish(target_gps_msg)

    def send_gps_to_drone(self, lat, lon, alt):
        """Envía la posición GPS al dron usando MAVLink."""
        self.connection.mav.set_position_target_global_int_send(
            0,  # Timestamp
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b110111111000,  # Ignorar velocidad y aceleración
            int(lat * 1e7),  # Latitud como entero
            int(lon * 1e7),  # Longitud como entero
            alt,  # Altitud deseada
            0, 0, 0,  # Velocidades
            0, 0, 0,  # Aceleraciones
            0, 0  # Yaw y tasa de yaw
        )
        self.get_logger().info(f"Enviando al dron: Lat={lat}, Lon={lon}, Alt={alt}m")

    def land_drone(self):
        """Método para aterrizar el dron cuando la batería esté baja."""
        # Aquí se envía el comando de aterrizaje usando MAVLink
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # Confirmación
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Comando de aterrizaje enviado.")

def main(args=None):
    """Función principal para inicializar y ejecutar el nodo."""
    rclpy.init(args=args)
    node = MissionControlNode()

    # Cambiar el modo a GUIDED antes de despegar y mover el dron
    node.set_mode('GUIDED')

    # Armar el dron
    node.arm_vehicle()

    # Despegar
    node.takeoff()

    # Esperar unos segundos para el despegue
    time.sleep(15)

    # Publicar las coordenadas de destino y mover el dron
    node.publish_target_gps(node.target_lat, node.target_lon, node.target_alt)

    # Monitorizar el estado de la batería
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


