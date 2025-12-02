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
        self.dist_seguridad = 0.65
        self.dist_critica = 0.25
        self.vel_avance = 0.35
        self.vel_giro = 0.6
        
        # --- NUEVAS VARIABLES DE ESTADO ---
        self.direccion_giro_fija = 0  # 0: Ninguna, 1: Izq, -1: Der
        self.tiempo_inicio_bloqueo = None # Para saber cuánto llevamos atascados
        
        self.get_logger().info('--- EXPLORADOR MEJORADO (ANTI-OSCILACIÓN) ---')

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        
        # Limpieza de datos
        ranges = np.where(ranges < 0.05, 10, ranges)
        ranges = np.where(ranges > 10, 10, ranges)
        
        total = len(ranges)
        chunk = total // 5
        
        # Sectores
        s_right  = np.min(ranges[0 : chunk])
        s_fright = np.min(ranges[chunk : 2*chunk])
        s_front  = np.min(ranges[2*chunk : 3*chunk])
        s_fleft  = np.min(ranges[3*chunk : 4*chunk])
        s_left   = np.min(ranges[4*chunk : total])
        
        min_global = np.min(ranges)

        twist = Twist()

        # --- LÓGICA DE COMPORTAMIENTO ---

        # 1. OBSTÁCULO FRONTAL (Detectado lejos)
        # Usamos 0.65 como umbral de "Parar y pensar"
        obstaculo_frente = (s_front < self.dist_seguridad or 
                            s_fleft < self.dist_seguridad/1.5 or 
                            s_fright < self.dist_seguridad/1.5)

        if obstaculo_frente:
            # A. Gestión del TIMEOUT (Si llevamos mucho tiempo parados)
            if self.tiempo_inicio_bloqueo is None:
                self.tiempo_inicio_bloqueo = time.time() # Empezamos a cronometrar
            
            tiempo_bloqueado = time.time() - self.tiempo_inicio_bloqueo
            
            # Si llevamos más de 2 segundos intentando girar y seguimos tapados...
            # O si estamos demasiado cerca (crítico)...
            # -> MARCHA ATRÁS
            if min_global < self.dist_critica or tiempo_bloqueado > 2.0:
                self.get_logger().warn(f'¡ATASCADO {tiempo_bloqueado:.1f}s! Retrocediendo...')
                twist.linear.x = -0.15
                twist.angular.z = 0.0 # Retroceder recto es más seguro a veces
                return self.pub.publish(twist) # Salimos de la función aquí

            # B. LÓGICA DE GIRO CON MEMORIA
            twist.linear.x = 0.0
            
            # Si NO teníamos una decisión tomada, la tomamos ahora
            if self.direccion_giro_fija == 0:
                espacio_izq = s_left + s_fleft
                espacio_der = s_right + s_fright
                
                if espacio_izq > espacio_der:
                    self.direccion_giro_fija = 1  # Decidimos Izquierda
                else:
                    self.direccion_giro_fija = -1 # Decidimos Derecha
            
            # Aplicamos el giro según la decisión MEMORIZADA
            # (Ignoramos si los sensores cambian un poco ahora)
            if self.direccion_giro_fija == 1:
                twist.angular.z = self.vel_giro
            else:
                twist.angular.z = -self.vel_giro

        else:
            # 2. CAMINO LIBRE (Reseteamos memoria)
            self.direccion_giro_fija = 0      # Olvidamos la dirección de giro
            self.tiempo_inicio_bloqueo = None # Reseteamos cronómetro
            
            # Navegación en pasillos (mantener centro)
            if s_left < 0.45:
                twist.linear.x = self.vel_avance * 0.8
                twist.angular.z = -0.4 
            elif s_right < 0.45:
                twist.linear.x = self.vel_avance * 0.8
                twist.angular.z = 0.4
            else:
                twist.linear.x = self.vel_avance
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