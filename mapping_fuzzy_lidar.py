"""
mapping_fuzzy_lidar.py

Ejemplo de construcción de un mapa de ocupación (occupancy grid)
en CoppeliaSim usando:

- Controlador de lógica borrosa (fuzzy.py) para mover el robot.
- Robot Pioneer P3DX (robotica.P3DX) con LiDAR activado.
- Asumimos acceso a la pose "perfecta" del robot (x, y, theta).

IMPORTANTE: 
- Ajusta los nombres de los métodos de robotica.P3DX para:
    - Obtener la pose:    robot.get_pose2d() --> (x, y, theta)
    - Obtener el LiDAR:   robot.get_lidar()  --> lista/array de distancias
- Ajusta también FOV y resolución del LiDAR a tu configuración real.
"""

import time
import math
import numpy as np

import robotica
import fuzzy  # tu controlador borroso


# =========================
# 1. Clase OccupancyGrid
# =========================

class OccupancyGrid:
    """
    Mapa de ocupación 2D en log-odds (grid regular).

    - El mapa representa un rectángulo en coordenadas del mundo
      [xmin, xmax] x [ymin, ymax].
    - resolution: tamaño de celda (m).
    - Internamente guarda log-odds (l), pero se puede convertir a p.
    """

    def __init__(self,
                 x_min=-5.0, x_max=5.0,
                 y_min=-5.0, y_max=5.0,
                 resolution=0.05):
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.resolution = resolution

        # Número de celdas en cada dirección
        self.nx = int((x_max - x_min) / resolution)
        self.ny = int((y_max - y_min) / resolution)

        # Log-odds inicial (p0 = 0.5 -> l0 = 0)
        self.grid = np.zeros((self.ny, self.nx), dtype=np.float32)

        # Parámetros de actualización (log-odds)
        # p_occ ~ 0.7, p_free ~ 0.3
        self.l_occ = math.log(0.7 / 0.3)
        self.l_free = math.log(0.3 / 0.7)

        # Saturación de log-odds para evitar overflow
        self.l_min = -4.0
        self.l_max = 4.0

    # ---------- Conversión entre mundo <-> grid ----------

    def world_to_grid(self, x, y):
        """
        Convierte coordenadas del mundo (x, y) [m] a índices (ix, iy) de la matriz.
        Devuelve (ix, iy) o None si está fuera del mapa.
        """
        ix = int((x - self.x_min) / self.resolution)
        iy = int((y - self.y_min) / self.resolution)

        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return ix, iy
        else:
            return None

    def grid_to_world(self, ix, iy):
        """
        Convierte índices (ix, iy) a coordenadas del mundo (x, y) en el centro de la celda.
        """
        x = self.x_min + (ix + 0.5) * self.resolution
        y = self.y_min + (iy + 0.5) * self.resolution
        return x, y

    # ---------- Actualización de celdas ----------

    def _update_cell(self, ix, iy, delta_l):
        """
        Suma delta_l al log-odds de la celda (ix, iy), con saturación.
        """
        if ix is None or iy is None:
            return
        self.grid[iy, ix] = np.clip(self.grid[iy, ix] + delta_l,
                                    self.l_min, self.l_max)

    def update_from_scan(self, x, y, theta,
                         ranges,
                         angle_min=-math.pi / 2,
                         angle_max=+math.pi / 2,
                         max_range=5.0):
        """
        Actualiza el mapa a partir de un escaneo LiDAR:

        - (x, y, theta): pose del robot en el mundo [m, rad].
        - ranges: lista/array con las distancias medidas (una por rayo).
        - angle_min, angle_max: FOV del LiDAR RELATIVO al robot (rad).
        - max_range: alcance máximo del LiDAR (m).

        Estrategia:
        - Para cada rayo:
            - Recorremos a lo largo del rayo en pasos de 'resolution'
              marcando celdas como libres.
            - Si la medida < max_range: marcamos la celda final como ocupada.
        """

        n = len(ranges)
        if n == 0:
            return

        angle_inc = (angle_max - angle_min) / max(n - 1, 1)

        for i, r in enumerate(ranges):
            # Ángulo absoluto del rayo
            angle = theta + angle_min + i * angle_inc

            # Limitamos r al máximo
            r_hit = min(r, max_range)

            # Número de pasos a lo largo del rayo
            n_steps = int(r_hit / self.resolution)

            # 1) Celdas libres a lo largo del rayo
            for s in range(n_steps):
                d = s * self.resolution
                x_s = x + d * math.cos(angle)
                y_s = y + d * math.sin(angle)
                idx = self.world_to_grid(x_s, y_s)
                if idx is None:
                    break  # nos salimos del mapa
                ix, iy = idx
                self._update_cell(ix, iy, self.l_free)

            # 2) Celda ocupada si de verdad hemos chocado con algo
            if r < max_range * 0.99:  # umbral para distinguir "infinito" de obstáculo
                x_end = x + r * math.cos(angle)
                y_end = y + r * math.sin(angle)
                idx_end = self.world_to_grid(x_end, y_end)
                if idx_end is not None:
                    ix_end, iy_end = idx_end
                    self._update_cell(ix_end, iy_end, self.l_occ)

    # ---------- Utilidades ----------

    def get_probability_map(self):
        """
        Devuelve una matriz con p(ocupado) en [0, 1] a partir de log-odds.
        """
        # p = 1 / (1 + exp(-l))
        return 1.0 / (1.0 + np.exp(-self.grid))


# ============================================
# 2. Bucle principal: fuzzy + LiDAR + mapping
# ============================================

def read_sonar_normalized(robot):
    """
    EJEMPLO: función que lee sonares del P3DX, los normaliza a [0, 1] y
    devuelve (dist_left, dist_center, dist_right).

    ADÁPTALO a cómo tengas implementada la lectura de sonares en robotica.py
    y cómo lo usas en fuzzy.py.
    """
    # Aquí asumo métodos de ejemplo, cámbialos por los tuyos reales:
    # distances = robot.get_sonar()  # p.ej. lista de 8 sonares [m]

    distances = robot.get_sonar()  # <-- AJUSTA ESTE NOMBRE

    # Ejemplo sencillo: agrupar sensores
    # (esto es orientativo; usa la misma lógica que ya usas en fuzzy.py)
    left = min(distances[5:8])      # sonares izquierda
    center = min(distances[2:5])    # frontales
    right = min(distances[0:3])     # derecha

    # Normalizamos a [0, 1] suponiendo rango [0, max_d]
    max_d = 2.0  # m, ajusta según tu escena
    dist_left = max(0.0, min(left / max_d, 1.0))
    dist_center = max(0.0, min(center / max_d, 1.0))
    dist_right = max(0.0, min(right / max_d, 1.0))

    return dist_left, dist_center, dist_right


def main(args=None):
    # 1) Conexión con CoppeliaSim
    coppelia = robotica.Coppelia()
    # Activa LiDAR y sonares (ajusta flags según tu constructor real)
    robot = robotica.P3DX(coppelia.sim,
                          'PioneerP3DX',
                          use_lidar=True)

    # 2) Crear mapa de ocupación
    og = OccupancyGrid(
        x_min=-5.0, x_max=5.0,
        y_min=-5.0, y_max=5.0,
        resolution=0.05
    )

    # Parámetros del LiDAR (AJUSTAR a tu configuración real en Coppelia)
    lidar_fov_min = -math.pi / 2   # -90º
    lidar_fov_max = +math.pi / 2   # +90º
    lidar_max_range = 5.0          # m

    # 3) Arrancar simulación
    coppelia.start_simulation()
    print("[INFO] Simulación iniciada.")

    t0 = time.time()
    max_sim_time = 60.0  # segundos de simulación real

    try:
        while time.time() - t0 < max_sim_time:
            # 3.1) Leer sonares y aplicar controlador fuzzy
            dist_left, dist_center, dist_right = read_sonar_normalized(robot)

            # Asumimos que en fuzzy.py tienes:
            #   avoid_simulator = ctrl.ControlSystemSimulation(avoid_ctrl)
            fuzzy.avoid_simulator.input['dist_left'] = dist_left
            fuzzy.avoid_simulator.input['dist_center'] = dist_center
            fuzzy.avoid_simulator.input['dist_right'] = dist_right
            fuzzy.avoid_simulator.compute()

            v_base = fuzzy.avoid_simulator.output['base_speed']
            v_diff = fuzzy.avoid_simulator.output['speed_diff']

            lspeed = v_base - v_diff / 2.0
            rspeed = v_base + v_diff / 2.0

            robot.set_speed(lspeed, rspeed)

            # 3.2) Leer pose y LiDAR
            # ADAPTAR ESTOS MÉTODOS a tu API real de robotica.P3DX
            x, y, theta = robot.get_pose2d()   # (m, m, rad)
            lidar_ranges = robot.get_lidar()   # lista/array de distancias [m]

            # 3.3) Actualizar mapa de ocupación
            og.update_from_scan(
                x, y, theta,
                lidar_ranges,
                angle_min=lidar_fov_min,
                angle_max=lidar_fov_max,
                max_range=lidar_max_range
            )

            # Pequeño sleep para no saturar el bucle
            time.sleep(0.05)

    finally:
        coppelia.stop_simulation()
        print("[INFO] Simulación detenida.")

        # Ejemplo: guardar mapa como npy (para visualizar luego en otro script)
        occ_prob = og.get_probability_map()
        np.save("occupancy_map.npy", occ_prob)
        print("[INFO] Mapa guardado en 'occupancy_map.npy'.")


if __name__ == '__main__':
    main()
