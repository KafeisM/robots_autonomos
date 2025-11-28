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
        self.resolution = 0.05
        self.map_size = 600
        self.center = self.map_size // 2
        
        # OFFSET CRÍTICO: Si las paredes se doblan al girar,
        # juega con este valor (0.15, 0.2, 0.25) hasta que se alineen.
        self.lidar_offset = 0.2 
        
        self.rango_max = 4.9 # Filtro de infinito
        
        # --- MAPA PROBABILÍSTICO ---
        # 0 = Libre, 50 = Desconocido, 100 = Ocupado
        # Usamos float para poder sumar decimales si queremos
        self.grid = np.full((self.map_size, self.map_size), 50.0, dtype=np.float32)
        
        # Pesos de probabilidad
        self.prob_libre = 5.0   # Cuánto "blanquea" un rayo de aire
        self.prob_ocupado = 30.0 # Cuánto "oscurece" un impacto
        
        self.get_logger().info('--- MAPEADOR V7 (PROBABILÍSTICO) ---')

    def odom_callback(self, msg):
        self.odom_ready = True
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        if not self.odom_ready: return
        
        sensor_x = self.px + (self.lidar_offset * math.cos(self.yaw))
        sensor_y = self.py + (self.lidar_offset * math.sin(self.yaw))
        
        start_x = int(sensor_x / self.resolution) + self.center
        start_y = int(sensor_y / self.resolution) + self.center
        
        ranges = msg.ranges
        angle = msg.angle_min
        step = 2 # Salto para optimizar velocidad
        
        for i in range(0, len(ranges), step):
            r = ranges[i]
            current_angle = self.yaw + angle + (i * msg.angle_increment)
            
            dist_real = r
            es_pared = True
            
            # Filtro de aire libre
            if r >= self.rango_max or r > 8.0:
                dist_real = self.rango_max
                es_pared = False
            elif r < 0.1:
                continue # Ruido
            
            # Coordenadas finales
            end_world_x = sensor_x + (dist_real * math.cos(current_angle))
            end_world_y = sensor_y + (dist_real * math.sin(current_angle))
            
            end_x = int(end_world_x / self.resolution) + self.center
            end_y = int(end_world_y / self.resolution) + self.center
            
            # --- RAYCASTING PROBABILÍSTICO ---
            puntos = self.bresenham(start_x, start_y, end_x, end_y)
            
            for (lx, ly) in puntos:
                if 0 <= lx < self.map_size and 0 <= ly < self.map_size:
                    # RESTAMOS oscuridad (tendencia a blanco)
                    self.grid[ly, lx] -= self.prob_libre

            # --- GOLPE DE PARED ---
            if es_pared and 0 <= end_x < self.map_size and 0 <= end_y < self.map_size:
                # SUMAMOS oscuridad (tendencia a negro)
                self.grid[end_y, end_x] += self.prob_ocupado
                # Vecinos para engrosar
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                         nx, ny = end_x + dx, end_y + dy
                         if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                            self.grid[ny, nx] += self.prob_ocupado

        # CLAMP (Mantener valores entre 0 y 100)
        # Esto es vital para que no se vaya a infinito
        np.clip(self.grid, 0, 100, out=self.grid)
        
        # Guardamos casteando a int para que ocupe menos y sea imagen válida
        np.save('occupancy_map.npy', self.grid.astype(np.uint8))

    def bresenham(self, x0, y0, x1, y1):
        points = []
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0: y += sy; err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0: x += sx; err += dy
                y += sy
        points.append((x, y))
        return points

def main():
    rclpy.init()
    node = MapeadorProbabilistico()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()