# Módulo de Gestión de Batería y Autonomía (`battery_manager.py`)

Nodo de **ROS 2 Humble** (`battery_manager_node`) encargado de monitorear en tiempo real el voltaje real de la batería LiPo del robot **MentorPi A1**, suavizar la señal ante picos de corriente de los motores, evaluar alertas operativas y estimar los minutos de autonomía restantes según la velocidad de desplazamiento.

---

## Características Principales

* **Lectura y Filtrado de Voltaje (EMA):** Se suscribe al voltaje crudo en milivolts (`/ros_robot_controller/battery`) y aplica un Filtro Promedio Móvil Exponencial ($\alpha = 0.15$) para evitar falsas caídas de porcentaje durante aceleraciones bruscas.
* **Conversión Lineal a Porcentaje:** Mapea el rango operativo de la batería (`7.0 V` = `0.0 %` hasta `8.4 V` = `100.0 %`).
* **Estimación Dinámica de Autonomía:** Calcula los minutos restantes combinando una tasa de descarga base en reposo (`1.67 %/min`, equivalente a ~60 min al 100 %) con un incremento proporcional a la velocidad lineal del robot (`15.0 %/min` por cada `m/s` recorrido), suscribiéndose a `/nexo/telemetry/speed`.
* **Evaluación de Estados Críticos:** Clasifica el estado energético en `OK`, `LOW_BATTERY_RETURN` ($\le 30.0\%$) y `CRITICAL_CHARGE_NOW` ($\le 20.0\%$) para habilitar bloqueos preventivos en el navegador.

---

## Interfaz ROS 2

### Tópicos Suscritos
| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/ros_robot_controller/battery` | `std_msgs/msg/UInt16` | Lectura cruda del voltaje de batería en milivolts (`mV`). |
| `/nexo/telemetry/speed` | `std_msgs/msg/Float32` | Velocidad lineal instantánea del robot (`m/s`) publicada por `location_manager`. |

### Tópicos Publicados (1 Hz)
| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/nexo/battery/percentage` | `std_msgs/msg/Float32` | Nivel de carga filtrado (`0.0` a `100.0 %`). |
| `/nexo/battery/voltage` | `std_msgs/msg/Float32` | Voltaje suavizado en Volts (`V`). |
| `/nexo/battery/autonomy_min` | `std_msgs/msg/Float32` | Tiempo estimado de autonomía restante en minutos (`min`). |
| `/nexo/battery/status` | `std_msgs/msg/String` | Estado operativo (`OK`, `LOW_BATTERY_RETURN`, `CRITICAL_CHARGE_NOW`). |

### Parámetros Configurables
* `min_operating_battery` (default: `30.0`): Umbral de batería baja (%).
* `critical_battery` (default: `20.0`): Umbral de batería crítica (%).
* `base_discharge_rate` (default: `1.67`): Tasa base de descarga en reposo (`%/min`).
* `motion_discharge_gain` (default: `15.0`): Factor de descarga adicional por velocidad (`%/min` por `m/s`).
* `v_empty` (default: `7.0`): Voltaje correspondiente al `0 %`.
* `v_full` (default: `8.4`): Voltaje correspondiente al `100 %`.

---

## Comandos de Compilación y Ejecución

### 1. Compilar el paquete
```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
colcon build --packages-select nexo_mentorpi
source install/setup.zsh
export ROS_DOMAIN_ID=0
```

### 2. Ejecutar el nodo `battery_manager`
```zsh
source /opt/ros/humble/setup.zsh
source ~/shared/nexo_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run nexo_mentorpi battery_manager
```

### 3. Verificar los tópicos publicados en otra terminal
```zsh
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
ros2 topic echo /nexo/battery/percentage --once
ros2 topic echo /nexo/battery/voltage --once
ros2 topic echo /nexo/battery/autonomy_min --once
ros2 topic echo /nexo/battery/status --once
```

---
