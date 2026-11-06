# Location Manager Node (`nexo_mentorpi`)

Este paquete contiene la implementación del nodo **Location Manager** para el robot **MentorPi A1** en ROS 2 Humble. Este nodo gestiona la localización en tiempo real del robot respecto al mapa global, la persistencia de puntos de interés (estaciones) en formato JSON y el envío de objetivos de navegación.

## Descripción del Nodo

El nodo `location_manager_node` realiza las siguientes funciones clave:

* **Consulta de TF2 (Localización Global):** Lee la transformación entre las tramas `map` y `base_footprint` a una frecuencia de **1 Hz** para obtener las coordenadas actuales ($x, y$) y la orientación ($\text{Yaw}$ en grados).
* **Publicación de Pose Actual:** Emite la ubicación del robot de forma continua en el tópico `/nexo/current_pose` (`geometry_msgs/msg/PoseStamped`).
* **Gestión de Puntos y Estaciones (JSON):**
  * **Guardado:** Al recibir un nombre en `/nexo/save_location`, almacena las coordenadas actuales en `~/shared/nexo_ws/src/nexo_mentorpi/config/locations.json`.
  * **Despacho de Metas:** Al recibir el nombre de una estación en `/nexo/get_location`, publica la meta correspondiente en `/goal_pose` para que Nav2 inicie la navegación.
* **Detección de Llegada:** Compara continuamente la distancia entre el robot y las estaciones guardadas. Si el robot se encuentra dentro de un radio de tolerancia de **0.25 metros**, publica el nombre de la estación en `/nexo/events/arrival`.

---

## Instrucciones de Ejecución

Sigue el orden estricto de las pestañas/terminales para asegurar que los transformadores TF2 y la odometría láser estén disponibles antes de iniciar el nodo de localización.

### Pestaña 1: Sistema de Navegación Nav2

Inicia el mapa y la localización de AMCL:

```zsh
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch navigation navigation.launch.py map:=map_equipoX
```
*(Reemplaza `map_equipoX` por el nombre de tu mapa guardado).*

---

### Pestaña 2: Corrección de Escaneo LiDAR (`scan_fixer`)

Ajusta la lectura del sensor láser para evitar discrepancias de filtrado:

```zsh
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
cd ~/shared && python3 scan_fixer.py
```

---

### Pestaña 3: Odometría Láser (`rf2o`)

Publica la odometría requerida por el árbol de transformaciones TF:

```zsh
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
ros2 launch ~/shared/rf2o_scan_fixed.launch.py
```

---

### Pestaña 4: Nodo Location Manager

Compila (si realizaste cambios) y ejecuta el nodo de localización:

```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
source install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run nexo_mentorpi location_manager
```

---

## Interacción con el Nodo por Tópicos

* **Guardar la posición actual como estación (ejemplo: "estacion_A"):**
  ```zsh
  ros2 topic pub --once /nexo/save_location std_msgs/msg/String "{data: 'estacion_A'}"
  ```

* **Mandar al robot hacia la estación guardada:**
  ```zsh
  ros2 topic pub --once /nexo/get_location std_msgs/msg/String "{data: 'estacion_A'}"
  ```