import numpy as np
import matplotlib.pyplot as plt
import time

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8)) # Hacemos la ventana más grande

print("Visualizador con ZOOM listo...")

# AJUSTE DE ZOOM: Cuanto menor sea este número, más cerca se verá
# 150 pixels = 7.5 metros a la redonda (con resolución 0.05)
RADIO_ZOOM = 150 

while True:
    try:
        grid = np.load('occupancy_map.npy')
        ax.clear()
        
        # Dibujamos el mapa
        ax.imshow(grid, cmap='Greys', origin='lower', vmin=0, vmax=100)
        
        # Calculamos el centro
        center_x = grid.shape[1] // 2
        center_y = grid.shape[0] // 2
        
        # Marcamos el centro (inicio del robot)
        ax.plot(center_x, center_y, 'rx', markersize=10) 
        
        # --- APLICAMOS EL ZOOM ---
        # Limitamos los ejes X e Y alrededor del centro
        ax.set_xlim(center_x - RADIO_ZOOM, center_x + RADIO_ZOOM)
        ax.set_ylim(center_y - RADIO_ZOOM, center_y + RADIO_ZOOM)
        
        ax.set_title(f"Mapa Log-Odds (Zoom: {RADIO_ZOOM}px)")
        
        # Quitamos los ejes numéricos para que se vea más limpio (opcional)
        ax.axis('off') 
        
        plt.draw()
        plt.pause(0.5)
        
    except FileNotFoundError:
        time.sleep(1)
    except Exception as e:
        print(f"Error: {e}")
        time.sleep(1)