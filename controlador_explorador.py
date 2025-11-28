import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time

class ExploradorAutonomo(Node):
    def __init__(self):
        super().__init__('explorador_node')
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- PARÁMETROS ---
        self.dist_seguridad = 0.65  # Distancia para empezar a evitar
        self.dist_critica = 0.25    # Distancia de "PÁNICO" (Retroceder)
        self.vel_avance = 0.35
        self.vel_giro = 0.6
        
        # Estado para evitar oscilaciones
        self.girando = False
        
        self.get_logger().info('--- EXPLORADOR ROBUSTO V2 (CON MARCHA ATRÁS) ---')

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Limpieza de datos (Infinitos y ceros)
        ranges = np.where(ranges < 0.05, 10, ranges) # Ruido cercano
        ranges = np.where(ranges > 10, 10, ranges)   # Infinitos
        
        # --- VISIÓN EN 5 SECTORES ---
        # Esto permite detectar mejor las esquinas que solo con 3
        total = len(ranges)
        chunk = total // 5
        
        # Definimos sectores (de derecha a izquierda del robot)
        # S1: Derecha Extrema, S2: Diagonal Der, S3: Frente, S4: Diagonal Izq, S5: Izquierda Extrema
        s_right      = np.min(ranges[0 : chunk])
        s_fright     = np.min(ranges[chunk : 2*chunk])
        s_front      = np.min(ranges[2*chunk : 3*chunk])
        s_fleft      = np.min(ranges[3*chunk : 4*chunk])
        s_left       = np.min(ranges[4*chunk : total])
        
        # Mínimo global para detectar colisiones inminentes
        min_global = np.min(ranges)

        twist = Twist()

        # --- LÓGICA DE COMPORTAMIENTO (PRIORIDADES) ---
        
        # 1. EMERGENCIA: ¿Estamos pegados a algo? -> MARCHA ATRÁS
        # Esto saca al robot de esquinas o bloqueos físicos
        if min_global < self.dist_critica:
            self.get_logger().warn(f'¡ATASCADO! Retrocediendo. Dist: {min_global:.2f}')
            twist.linear.x = -0.15  # Retroceder despacio
            # Giramos un poco mientras retrocedemos para desencajarnos
            if s_left < s_right:
                twist.angular.z = -0.3 # Cola a la izquierda
            else:
                twist.angular.z = 0.3  # Cola a la derecha
            self.girando = True # Forzamos estado de giro posterior

        # 2. OBSTÁCULO FRONTAL: ¿Pared enfrente? -> GIRAR
        elif s_front < self.dist_seguridad or s_fleft < self.dist_seguridad/1.5 or s_fright < self.dist_seguridad/1.5:
            twist.linear.x = 0.0
            
            # Decisión inteligente de giro: Ir hacia el lado más abierto
            # Sumamos los sectores de cada lado para ver cual tiene más "aire" total
            espacio_izq = s_left + s_fleft
            espacio_der = s_right + s_fright
            
            if espacio_izq > espacio_der:
                twist.angular.z = self.vel_giro  # Girar Izquierda
            else:
                twist.angular.z = -self.vel_giro # Girar Derecha
            
            self.girando = True

        # 3. NAVEGACIÓN EN PASILLOS (Centrado)
        # Si no hay obstáculo delante, pero estamos muy cerca de una pared lateral
        elif s_left < 0.45:
            # Muy cerca izquierda -> Corregir suave a derecha
            twist.linear.x = self.vel_avance * 0.8 # Bajamos velocidad un poco
            twist.angular.z = -0.4 
            self.girando = False
            
        elif s_right < 0.45:
            # Muy cerca derecha -> Corregir suave a izquierda
            twist.linear.x = self.vel_avance * 0.8
            twist.angular.z = 0.4
            self.girando = False

        # 4. CAMINO LIBRE -> AVANZAR
        else:
            twist.linear.x = self.vel_avance
            twist.angular.z = 0.0
            self.girando = False
            
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