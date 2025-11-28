import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class ExploradorAutonomo(Node):
    def __init__(self):
        super().__init__('explorador_node')
        
        # Suscripción y Publicación
        self.sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parámetros de navegación
        self.distancia_seguridad = 0.6  # Metros
        self.velocidad_avance = 0.3
        self.velocidad_giro = 0.5
        
        self.get_logger().info('--- INICIANDO EXPLORACIÓN PARA MAPEADO ---')

    def laser_callback(self, msg):
        # Procesamos los datos del láser (Pioneer tiene 180 grados aprox)
        ranges = np.array(msg.ranges)
        
        # Limpiamos ceros e infinitos
        ranges = np.where(ranges < 0.1, 10, ranges)
        ranges = np.where(ranges > 10, 10, ranges)
        
        # Dividimos la visión en 3 sectores
        total = len(ranges)
        derecha = min(ranges[0 : total//3])
        centro = min(ranges[total//3 : 2*total//3])
        izquierda = min(ranges[2*total//3 : total])
        
        twist = Twist()
        
        # LÓGICA DE EXPLORACIÓN (Braitenberg Modificado)
        # 1. ¿Hay obstáculo enfrente?
        if centro < self.distancia_seguridad:
            self.get_logger().info(f'¡Pared! Girando. (I:{izquierda:.1f} D:{derecha:.1f})')
            twist.linear.x = 0.0
            # Girar hacia donde haya más espacio
            if izquierda > derecha:
                twist.angular.z = self.velocidad_giro # Girar izq
            else:
                twist.angular.z = -self.velocidad_giro # Girar der
        
        # 2. ¿Estamos muy pegados a un lado? (Corrección de trayectoria)
        elif izquierda < 0.4:
            twist.linear.x = 0.15
            twist.angular.z = -0.3 # Alejarse de la pared izquierda
        elif derecha < 0.4:
            twist.linear.x = 0.15
            twist.angular.z = 0.3  # Alejarse de la pared derecha
            
        # 3. Camino libre
        else:
            twist.linear.x = self.velocidad_avance
            twist.angular.z = 0.0
            
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ExploradorAutonomo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()