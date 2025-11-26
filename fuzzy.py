'''
Controlador de lógica borrosa

Este módulo implementa un controlador de lógica borrosa robusto y sin
contradicciones para el robot Pioneer P3DX en CoppeliaSim, orientado a
deambular (wander) y evitar colisiones de forma reactiva y suave.
- Entradas (difusas): dist_left, dist_center, dist_right ∈ [0, 1]
    (0 = muy cerca/choque, 1 = muy lejos/libre)
- Salidas (difusas): base_speed ∈ [0, 2], speed_diff ∈ [-2, 2]
    (base_speed = velocidad media; speed_diff > 0 gira a izquierda,
     speed_diff < 0 gira a derecha)
- Reglas: prioridad a evitar choque frontal; luego evitar laterales; por
    defecto avanzar recto rápido si todo está libre.
'''

import robotica
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time

# -------------------------------------------------------------------
# --- Configuración del controlador de lógica borrosa
# -------------------------------------------------------------------

# 1. ANTECEDENTES (Entradas)
# Tres distancias normalizadas a obstáculos (0 = muy cerca, 1 = libre)
dist_left = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'dist_left')
dist_center = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'dist_center')
dist_right = ctrl.Antecedent(np.arange(0, 1.01, 0.01), 'dist_right')

# 2. CONSECUENTES (Salidas) - Enfoque simple y directo
# Controlamos la velocidad base y el diferencial entre ruedas.
# Positivo -> gira a la izquierda; Negativo -> gira a la derecha.
speed_diff = ctrl.Consequent(np.arange(-2, 2.01, 0.01), 'speed_diff')
base_speed = ctrl.Consequent(np.arange(0, 2.01, 0.01), 'base_speed')

# 3. FUNCIONES DE PERTENENCIA
# Entradas: se simplifican a dos estados (cerca / lejos) para decisiones claras.
# trapmf = función trapezoidal: [a, b, c, d]
#  - altísima pertenencia a "cerca" para valores bajos (zona 0..~0.3-0.5)
#  - altísima pertenencia a "lejos" para valores altos (zona ~0.6..1)
dist_left['cerca'] = fuzz.trapmf(dist_left.universe, [0, 0, 0.3, 0.5])
dist_left['lejos'] = fuzz.trapmf(dist_left.universe, [0.4, 0.6, 1, 1])
dist_center['cerca'] = fuzz.trapmf(dist_center.universe, [0, 0, 0.4, 0.6])
dist_center['lejos'] = fuzz.trapmf(dist_center.universe, [0.5, 0.7, 1, 1])
dist_right['cerca'] = fuzz.trapmf(dist_right.universe, [0, 0, 0.3, 0.5])
dist_right['lejos'] = fuzz.trapmf(dist_right.universe, [0.4, 0.6, 1, 1])

# Salidas: base_speed y speed_diff usan funciones triangulares (trimf)
#  - base_speed: "lenta" para reacción segura, "rápida" para avance normal
#  - speed_diff: negativo = giro derecha, 0 = recto, positivo = giro izquierda
base_speed['lenta'] = fuzz.trimf(base_speed.universe, [0, 0.5, 1.0])
base_speed['rapida'] = fuzz.trimf(base_speed.universe, [0.8, 1.5, 2.0])

speed_diff['fuerte_derecha'] = fuzz.trimf(speed_diff.universe, [-2.0, -1.5, -1.0])
speed_diff['recto'] = fuzz.trimf(speed_diff.universe, [-0.5, 0, 0.5])
speed_diff['fuerte_izquierda'] = fuzz.trimf(speed_diff.universe, [1.0, 1.5, 2.0])

# 4. BASE DE REGLAS
# Prioridad más alta: evitar colisión frontal
rule1 = ctrl.Rule(dist_center['cerca'], (base_speed['lenta'], speed_diff['fuerte_izquierda']))
# Si el frente está despejado, evaluar laterales
rule2 = ctrl.Rule(dist_center['lejos'] & dist_left['cerca'], (base_speed['rapida'], speed_diff['fuerte_derecha']))
rule3 = ctrl.Rule(dist_center['lejos'] & dist_right['cerca'], (base_speed['rapida'], speed_diff['fuerte_izquierda']))
# Comportamiento por defecto: avanzar recto
rule4 = ctrl.Rule(dist_center['lejos'] & dist_left['lejos'] & dist_right['lejos'], (base_speed['rapida'], speed_diff['recto']))

# 5. CREACIÓN DEL SISTEMA DE CONTROL
avoid_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
avoid_simulator = ctrl.ControlSystemSimulation(avoid_ctrl)

# -------------------------------------------------------------------
# --- Bucle principal de control del robot
# -------------------------------------------------------------------

def main(args=None):
    """
    Arranca la simulación en CoppeliaSim, lee los sensores de sonar del
    Pioneer P3DX, evalúa el controlador borroso y aplica las velocidades
    calculadas en las ruedas. 

    Mapeo de los sonares a zonas:
    - izquierda:  sensores [5, 6, 7]
    - centro:     sensores [3, 4]
    - derecha:    sensores [0, 1, 2]

    Conversión de salidas a velocidades de rueda:
    - v_base es la velocidad media.
    - v_diff crea el giro: >0 izquierda, <0 derecha.
      lspeed = v_base - v_diff/2
      rspeed = v_base + v_diff/2
    """
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()

    while coppelia.is_running():
        # 1) Lectura de sensores (valores normalizados 0..1)
        readings = robot.get_sonar()
        
        # 2) Agregación por zonas usando el mínimo (el obstáculo más cercano)
        input_left = min(readings[5], readings[6], readings[7])
        input_center = min(readings[3], readings[4])
        input_right = min(readings[0], readings[1], readings[2])

        # 3) Asignar entradas al simulador borroso
        avoid_simulator.input['dist_left'] = input_left
        avoid_simulator.input['dist_center'] = input_center
        avoid_simulator.input['dist_right'] = input_right

        # 4) Inferencia y defuzzificación
        avoid_simulator.compute()

        # 5) Recuperar salidas nítidas (crisp)
        v_base = avoid_simulator.output['base_speed']
        v_diff = avoid_simulator.output['speed_diff']

        # 6) Convertir velocidad base y diferencial a velocidades de ruedas
        # Modelo estable: v_base = velocidad media; v_diff = giro
        lspeed = v_base - v_diff / 2
        rspeed = v_base + v_diff / 2
        
        # 7) Aplicar velocidades y esperar un pequeño intervalo
        robot.set_speed(lspeed, rspeed)
        time.sleep(0.05)

    coppelia.stop_simulation()


if __name__ == '__main__':
    main()