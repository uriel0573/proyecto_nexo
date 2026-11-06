# Smac Navigator Node (`smac_navigator`)

Este paquete contiene la implementación del nodo **Smac Navigator** para el robot **MentorPi A1** en ROS 2 Humble. Este nodo actúa como la capa de control de navegación autónoma con validación de seguridad por batería e integración con **Nav2 (Smac Planner / Hybrid A*)**.

---

## Descripción del Nodo

El nodo `smac_navigator_node` gestiona la autonomía y navegación del robot realizando las siguientes funciones:

* **Integración con Nav2 (`BasicNavigator`):** Interfaz simplificada con la pila de navegación de ROS 2 para despachar metas en el mapa global utilizando el algoritmo de planificación cinemática **Smac Planner (Hybrid A*)** con modelo Dubins.
* **Filtro de Seguridad por Batería:** Se suscribe a los tópicos del `battery_manager` (`/nexo/battery/status` y `/nexo/battery/percentage`). Si el nivel de energía entra en estado de advertencia (`LOW_BATTERY_RETURN`) o crítico (`CRITICAL_CHARGE_NOW`), **cancela y rechaza** cualquier orden de navegación para proteger la integridad del robot.
* **Control mediante Poses Remotas (Estado Actual):** Intercepta automáticamente los objetivos enviados desde la interfaz gráfica de **Foxglove Studio** a través del tópico `/goal_pose` (`geometry_msgs/msg/PoseStamped`).

---

## Estado Actual vs. Próxima Etapa

> ⚠️ **Nota de Arquitectura:**  
> **Estado Actual:** La navegación opera mediante la selección interactiva de metas puntuales (**Poses**) marcadas manualmente en la interfaz 3D de **Foxglove Studio**.  
> **Próxima Actualización:** Se migrará a un esquema de selección de **Estaciones del Laboratorio** (puntos de interés predefinidos en `locations.json` mediante `location_manager`), limitando los desplazamientos exclusivamente a las zonas operativas autorizadas.

---

## Instrucciones de Ejecución

Para iniciar la navegación autónoma completa, ejecuta los comandos en el orden indicado a través de **5 pestañas de terminal independientes**:

### Pestaña 1: Navegación Nav2 + Configuración Smac Planner
Inicia el sistema de mapas, AMCL y la pila de Nav2 cargando el mapa del laboratorio y los parámetros del planificador Smac:

```zsh
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

ros2 launch navigation navigation.launch.py \
  map:=map_equipo3_salon \
  params_file:=/home/pi/docker/tmp/nexo_ws/src/nexo_mentorpi/config/nav2_params.yaml
```

### Pestaña 2: Corrección de Escaneo LiDAR (`scan_fixer`)
Filtra y ajusta las lecturas del sensor láser para evitar inconsistencias de rango:

```zsh
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
cd ~/shared && python3 scan_fixer.py
```

### Pestaña 3: Odometría Láser (`rf2o`)
Publica la odometría planar calculada por correlación de lecturas del escáner LiDAR:

```zsh
source /opt/ros/humble/setup.zsh
export ROS_DOMAIN_ID=0
ros2 launch ~/shared/rf2o_scan_fixed.launch.py
```

### Pestaña 4: Servidor WebSocket para Foxglove (`rosbridge_server`)
Abre el canal de comunicación WebSocket para conectar la interfaz visual en tu computadora:

```zsh
source /opt/ros/humble/setup.zsh
source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### Pestaña 5: Retransmisión de Trayectorias y Nodos de Control
Inicia el repetidor de rutas, la supervisión de batería y el nodo navegador:

```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
source install/setup.zsh
export ROS_DOMAIN_ID=0

# Retransmisión de trayectoria para visualización
cd ~/shared && python3 path_relay.py &

# Administrador de batería en segundo plano
ros2 run nexo_mentorpi battery_manager &

# Nodo principal de navegación Smac
ros2 run nexo_mentorpi smac_navigator
```

---

## Operación desde Foxglove Studio

1. Abre **Foxglove Studio** en tu PC y conéctate por **Rosbridge (WebSocket)** en la dirección `ws://<IP_DE_TU_ROBOT>:9090`.
2. Asigna la estimación inicial del robot en el mapa usando **2D Pose Estimate**.
3. Usa la herramienta **"Publish Goal"** (configurada al tópico `/goal_pose`) para hacer clic en el mapa y enviar un objetivo de desplazamiento al nodo.