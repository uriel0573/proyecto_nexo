# Módulo de Localización, Cinemática y Gestión de Estaciones (`location_manager.py`)

Nodo de **ROS 2 Humble** (`location_manager_node`) encargado de consultar en tiempo real la pose global del robot **MentorPi A1** en el mapa mediante TF2 (`map -> base_footprint`), calcular su velocidad lineal y distancia recorrida acumulada, y gestionar el guardado y detección de arribo a estaciones de trabajo en `config/locations.json`.

---

## Características Principales

* **Localización Global por TF2:** Consulta cada `1.0 s` (`1 Hz`) la transformada `map -> base_footprint` y convierte la orientación de cuaternión a ángulo `yaw` en grados sexagesimales.
* **Cálculo Cinemático (Velocidad y Distancia):** Integra los desplazamientos euclidianos entre lecturas sucesivas para publicar la velocidad lineal instantánea (`m/s`) y la distancia total recorrida (`m`).
* **Filtro de Ruido y Saltos de Relocalización:** Implementa una banda de desplazamiento válido (`0.02 m <= step <= 1.5 m`):
  * Descarta micro-variaciones menores a `2 cm` (`min_step = 0.02 m`) generadas por el ruido de la odometría láser (`rf2o`) cuando el robot está detenido[cite: 1].
  * Ignora saltos mayores a `1.5 m` en un solo segundo para evitar que una reubicación manual en Foxglove (*Publish 2D pose estimate*) sume distancia falsa.
* **Detección Automática de Arribo:** Compara la posición actual contra las coordenadas registradas en `locations.json` y publica un evento en `/nexo/events/arrival` cuando el robot entra en un radio de `0.25 m` de una estación.

---

## Interfaz ROS 2

### Tópicos Suscritos
| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/nexo/save_location` | `std_msgs/msg/String` | Guarda las coordenadas actuales $(x, y, \text{yaw})$ en `locations.json` bajo el nombre enviado. |
| `/nexo/get_location` | `std_msgs/msg/String` | Busca una estación por nombre en `locations.json` y publica su pose hacia `/goal_pose`. |
| `/nexo/telemetry/reset_distance` | `std_msgs/msg/String` | Reinicia el contador de distancia recorrida acumulada a `0.0 m`. |

### Tópicos Publicados (1 Hz y Eventos)
| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/nexo/current_pose` | `geometry_msgs/msg/PoseStamped` | Pose actual del robot en el marco `map`. |
| `/nexo/telemetry/speed` | `std_msgs/msg/Float32` | Velocidad lineal instantánea del robot (`m/s`). |
| `/nexo/telemetry/distance` | `std_msgs/msg/Float32` | Distancia recorrida acumulada (`m`). |
| `/nexo/events/arrival` | `std_msgs/msg/String` | Nombre de la estación alcanzada al entrar en su radio de tolerancia (`0.25 m`). |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Meta de navegación enviada cuando se solicita una estación vía `/nexo/get_location`. |

### Parámetros Configurables
* `min_step` (default: `0.02`): Desplazamiento mínimo en metros para contabilizar movimiento y velocidad[cite: 1].

---

## Comandos de Compilación y Ejecución

### 1. Requisitos previos activos
Para que exista la transformada `map -> base_footprint`, deben estar corriendo Nav2 (`navigation.launch.py`), `scan_fixer.py` y `rf2o_scan_fixed.launch.py`, con el robot localizado en el mapa[cite: 1].

### 2. Compilar el paquete
```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
colcon build --packages-select nexo_mentorpi
source install/setup.zsh
export ROS_DOMAIN_ID=0
```

### 3. Ejecutar el nodo `location_manager`
```zsh
source /opt/ros/humble/setup.zsh
source ~/shared/nexo_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run nexo_mentorpi location_manager
```

### 4. Comandos útiles de prueba y telemetría
* **Consultar pose, velocidad y distancia en tiempo real:**
  ```zsh
  source /opt/ros/humble/setup.zsh
  export ROS_DOMAIN_ID=0
  ros2 topic echo /nexo/current_pose --once
  ros2 topic echo /nexo/telemetry/speed --once
  ros2 topic echo /nexo/telemetry/distance --once
  ```
* **Reiniciar la distancia recorrida acumulada a `0.0 m`:**
  ```zsh
  source /opt/ros/humble/setup.zsh
  export ROS_DOMAIN_ID=0
  ros2 topic pub --once /nexo/telemetry/reset_distance std_msgs/msg/String "{data: 'reset'}"
  ```
* **Guardar la posición actual del robot como una estación:**
  ```zsh
  source /opt/ros/humble/setup.zsh
  export ROS_DOMAIN_ID=0
  ros2 topic pub --once /nexo/save_location std_msgs/msg/String "{data: 'Est_Prueba'}"
