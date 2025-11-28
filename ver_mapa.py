import numpy as np
import matplotlib.pyplot as plt
import time

plt.ion()
fig, ax = plt.subplots()

print("Visualizador V4 listo...")

while True:
    try:
        grid = np.load('occupancy_map.npy')
        ax.clear()
        
        # vmin=0 (Blanco), vmax=100 (Negro)
        # Usamos mapa de colores invertido para que 100 sea negro y 0 blanco
        ax.imshow(grid, cmap='Greys', origin='lower', vmin=0, vmax=100)
        
        # Marcamos el centro (inicio)
        c = grid.shape[0] // 2
        ax.plot(c, c, 'rx') 
        
        ax.set_title("Mapa Raycasting (Blanco=Libre, Negro=Pared)")
        plt.draw()
        plt.pause(0.5)
        
    except FileNotFoundError:
        time.sleep(1)
    except Exception:
        pass