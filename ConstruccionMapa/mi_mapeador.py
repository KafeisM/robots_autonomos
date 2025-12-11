import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class MapeadorProbabilistico(Node):
    def __init__(self):
        super().__init__('mapeador_probabilistico')
        
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.px = 0.0; self.py = 0.0; self.yaw = 0.0
        self.odom_ready = False
        
        # --- CONFIGURACIÓN ---
        self.resolution = 0.05  # 5 cm por pixel (puedes bajar a 0.02 para más nitidez)
        self.map_size = 800     # Aumentado un poco para tener margen
        self.center = self.map_size // 2
        
        # OFFSET IMPORTANTE: Ajusta esto según la posición real del Lidar en el robot.
        # En el Pioneer P3DX suele estar adelantado unos 20cm del centro de giro.
        self.lidar_offset = 0.0
        
        self.rango_max = 5.0 # Límite del sensor Hokuyo simulado
        
        # --- MAPA LOG-ODDS ---
        # Inicializamos en 0.0 (que representa Probabilidad = 0.5, Desconocido)
        self.map_log = np.zeros((self.map_size, self.map_size), dtype=np.float32)
        
        # Valores Log-Odds (Ajustables)
        # l_occ = log(p_occ / (1 - p_occ))
        # Si p_occ = 0.8 -> l_occ ≈ 1.38
        # Si p_free = 0.3 -> l_free ≈ -0.84
        self.l_occ = 2.0    # Valor alto para marcar pared rápido
        self.l_free = -0.7  # Valor bajo para limpiar espacio suavemente
        
        # Límites para evitar overflow (saturación)
        self.l_max = 10.0
        self.l_min = -10.0
        
        self.get_logger().info('--- MAPEADOR V8 (LOG-ODDS PRO) ---')

    def odom_callback(self, msg):
        self.odom_ready = True
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Conversión Cuaternio a Euler (Yaw)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if not self.odom_ready: return
        
        # Posición del sensor en el mundo
        sensor_x = self.px + (self.lidar_offset * math.cos(self.yaw))
        sensor_y = self.py + (self.lidar_offset * math.sin(self.yaw))
        
        # Índices del sensor en la matriz
        start_c = int(sensor_x / self.resolution) + self.center
        start_r = int(sensor_y / self.resolution) + self.center
        
        ranges = msg.ranges
        angle_min = msg.angle_min
        inc = msg.angle_increment
        
        # Usamos paso 1 o 2. Con paso 1 tendrás más detalle pero más coste CPU.
        step = 2 
        
        for i in range(0, len(ranges), step):
            r = ranges[i]
            
            # Filtro básico de rango válido
            if r < 0.1: continue
            
            # Determinamos si es obstáculo o max range
            es_obstaculo = True
            dist_eff = r
            
            if r >= self.rango_max or r >= 4.95: 
                es_obstaculo = False
                dist_eff = self.rango_max - 0.1 # Ajustamos para limpiar hasta casi el final
            
            # Ángulo actual del rayo
            current_angle = self.yaw + angle_min + (i * inc)
            
            # Punto final en el mundo
            end_world_x = sensor_x + (dist_eff * math.cos(current_angle))
            end_world_y = sensor_y + (dist_eff * math.sin(current_angle))
            
            end_c = int(end_world_x / self.resolution) + self.center
            end_r = int(end_world_y / self.resolution) + self.center
            
            # --- RAYCASTING (Bresenham) ---
            rr, cc = self.line(start_r, start_c, end_r, end_c)
            
            # Verificamos límites de matriz
            valid_mask = (rr >= 0) & (rr < self.map_size) & (cc >= 0) & (cc < self.map_size)
            rr = rr[valid_mask]
            cc = cc[valid_mask]
            
            if len(rr) == 0: continue
            
            # --- MEJORA: PROTECCIÓN DE PAREDES ---
            
            # 1. Separamos el rayo: "Camino libre" vs "Punto de impacto"
            if es_obstaculo:
                # El camino libre es todo MENOS el último punto
                rr_free = rr[:-1]
                cc_free = cc[:-1]
                
                # El obstáculo es solo el último punto
                r_obst = rr[-1]
                c_obst = cc[-1]
            else:
                # Si no hubo impacto (max range), todo es libre
                rr_free = rr
                cc_free = cc
                r_obst = None
            
            # 2. ACTUALIZAR ESPACIO LIBRE (CON PROTECCIÓN)
            # Solo aplicamos "l_free" a celdas que NO sean ya muros fuertes.
            # Si una celda ya tiene valor > 1.0 (bastante seguro de ser muro), 
            # asumimos que este rayo que la atraviesa es un error de sensor y NO la borramos.
            if len(rr_free) > 0:
                current_vals = self.map_log[rr_free, cc_free]
                # Máscara: Solo actualizamos celdas "dudosas" o libres (valor < 2.0)
                mask_update = current_vals < 2.0 
                
                # Aplicamos la resta solo a las celdas que pasen el filtro
                self.map_log[rr_free[mask_update], cc_free[mask_update]] += self.l_free

            # 3. ACTUALIZAR OBSTÁCULO
            if r_obst is not None:
                if 0 <= r_obst < self.map_size and 0 <= c_obst < self.map_size:
                    # Aquí sumamos probabilidad de ocupación
                    self.map_log[r_obst, c_obst] += self.l_occ
                    
        # CLAMP GLOBAL (Evitar infinitos)
        np.clip(self.map_log, self.l_min, self.l_max, out=self.map_log)
        
        # --- GENERAR IMAGEN PARA GUARDAR ---
        # Convertimos Log-Odds a Probabilidad [0, 100]
        # p = 1 - (1 / (1 + exp(L)))
        # Mapeo rápido: L < 0 es libre (blanco), L > 0 ocupado (negro)
        
        # Fórmula sigmoide para suavidad
        probabilities = 1.0 - (1.0 / (1.0 + np.exp(self.map_log)))
        grid_export = (probabilities * 100).astype(np.uint8)
        
        np.save('occupancy_map.npy', grid_export)

    # Bresenham optimizado vectorizado (estilo skimage)
    def line(self, r0, c0, r1, c1):
        r0 = int(r0); c0 = int(c0); r1 = int(r1); c1 = int(c1)
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        
        if dr > dc:
            rr = np.linspace(r0, r1, dr + 1, dtype=np.int32)
            cc = np.linspace(c0, c1, dr + 1, dtype=np.int32) # Interpolación simple para velocidad
        else:
            rr = np.linspace(r0, r1, dc + 1, dtype=np.int32)
            cc = np.linspace(c0, c1, dc + 1, dtype=np.int32)
            
        return rr, cc

def main():
    rclpy.init()
    node = MapeadorProbabilistico()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()