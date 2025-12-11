import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np

class VisualizadorSLAM(Node):
    def __init__(self):
        super().__init__('visualizador_slam')
        
        # Nos suscribimos al mapa que genera el slam_toolbox
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        
        self.map_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.received = False
        
        self.get_logger().info('--- VISUALIZADOR DE SLAM INICIADO ---')
        self.get_logger().info('Esperando datos del tópico /map...')

    def map_callback(self, msg):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        
        # Los datos vienen en una lista 1D, hay que convertirlos a matriz 2D
        raw_data = np.array(msg.data, dtype=np.int8)
        
        # ROS usa -1 para desconocido, 0 para libre, 100 para ocupado
        # Vamos a "arreglar" los colores para que se vea bonito:
        # -1 (Desconocido) -> 50 (Gris)
        # 0 (Libre) -> 0 (Blanco)
        # 100 (Ocupado) -> 100 (Negro)
        
        grid = np.full_like(raw_data, 50) # Todo gris por defecto
        grid[raw_data == 0] = 0           # Lo libre a blanco
        grid[raw_data == 100] = 100       # Lo ocupado a negro
        
        # Reformar a 2D (Alto x Ancho)
        self.map_data = grid.reshape((self.height, self.width))
        self.received = True

def main():
    rclpy.init()
    node = VisualizadorSLAM()
    
    # Configuración de Matplotlib
    plt.ion() # Modo interactivo
    fig, ax = plt.subplots()
    ax.set_title("Mapa SLAM Toolbox (En vivo)")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            if node.received:
                ax.clear()
                # Mostramos el mapa
                # cmap='Greys': 0=Blanco (Libre), 100=Negro (Pared), 50=Gris
                # vmin/vmax aseguran que la escala de colores no baile
                ax.imshow(node.map_data, cmap='Greys', origin='lower', vmin=0, vmax=100)
                
                ax.set_title(f"SLAM en proceso... ({node.width}x{node.height} celdas)")
                ax.set_xlabel("Celdas X")
                ax.set_ylabel("Celdas Y")
                
                plt.draw()
                plt.pause(0.1) # Actualizar cada medio segundo
                node.received = False # Esperar al siguiente mapa
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()