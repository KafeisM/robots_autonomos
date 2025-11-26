import numpy as np
import matplotlib.pyplot as plt

occ_prob = np.load("occupancy_map.npy")

# Invertimos para que:
#   - celda muy ocupada (p≈1) → 1 - p ≈ 0 → negro
#   - celda libre (p≈0)       → 1 - p ≈ 1 → blanco
img = 1.0 - occ_prob

plt.figure()
plt.title("Occupancy grid (blanco = libre, negro = ocupado)")
plt.imshow(img, origin="lower", cmap="gray", vmin=0, vmax=1)
plt.xlabel("x (celdas)")
plt.ylabel("y (celdas)")
plt.show()
