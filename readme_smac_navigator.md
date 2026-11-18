# Módulo de Navegación Autónoma y Decisión Energética (`smac_navigator.py`)

Nodo interactivo de **ROS 2 Humble** (`smac_navigator`) construido sobre `nav2_simple_commander` (`BasicNavigator`) que gestiona el envío de metas de navegación entre las 8 estaciones del almacén (`map_equipo3_alamcen_ed`), respetando zonas prohibidas del mapa y validando previamente la viabilidad energética de cada trayectoria.

---

## Características Principales

* **Planificación de Rutas y Evasión de Zonas Restringidas:** Utiliza el servidor de planificación de Nav2 (`getPath` y `goToPose`) sobre el mapa editado `map_equipo3_alamcen_ed`, rodeando automáticamente las zonas bloqueadas y manteniendo los márgenes de seguridad del costmap local.
* **Validación Energética Previa al Viaje:** Antes de mover el robot, calcula la longitud real de la ruta en metros y estima el consumo energético aplicando un factor de descarga de $k = 4.0\text{ \%/m}$ más un margen de reserva de seguridad del `20.0 %`[cite: 1]:
  * **Destinos de Carga (`Est C1`, `Est C2`, `Est C3`):** Evalúa únicamente la energía necesaria para el trayecto de **ida** ($E_{\text{req}} = k \cdot L_{\text{ida}} + \text{Reserva}$).
  * **Destinos de Trabajo (`Alm 1`, `Alm 2`, `Alm 3`, `AGV Pool`, `Inicio`):** Evalúa el trayecto completo de **ida + regreso** hasta la estación de carga más cercana ($E_{\text{req}} = k \cdot (L_{\text{ida}} + L_{\text{regreso}}) + \text{Reserva}$)[cite: 1].
* **Redirección Inteligente a Cargadores Alcanzables:** Si la batería actual es insuficiente para completar el viaje solicitado (o si `battery_manager` reporta estado `LOW_BATTERY_RETURN` / `CRITICAL_CHARGE_NOW`), el nodo evalúa todas las estaciones de carga disponibles, muestra a cuáles sí alcanza a llegar el robot y ofrece redirigirlo automáticamente al cargador más cercano.

---

## Mapa de Estaciones Configuradas (`map_equipo3_alamcen_ed`)

| ID | Estación | Tipo | Coordenada X (m) | Coordenada Y (m) | Yaw (rad) |
| :-: | :--- | :--- | :---: | :---: | :---: |
| `1` | `Est C1` | Carga | `3.53` | `3.93` | `0.0` |
| `2` | `Est C2` | Carga | `0.39` | `-0.03` | `0.0` |
| `3` | `Est C3` | Carga | `8.04` | `-0.41` | `0.0` |
| `4` | `Alm 1` | Almacén | `0.22` | `2.87` | `0.0` |
| `5` | `Alm 2` | Almacén | `3.62` | `0.37` | `0.0` |
| `6` | `Alm 3` | Almacén | `9.19` | `1.03` | `0.0` |
| `7` | `AGV Pool` | Base / Pool | `0.00` | `0.00` | `0.0` |
| `8` | `Inicio` | Origen | `0.18` | `4.28` | `0.0` |

---

## Interfaz ROS 2

### Tópicos Suscritos
| Tópico | Tipo de Mensaje | Descripción |
| :--- | :--- | :--- |
| `/nexo/battery/percentage` | `std_msgs/msg/Float32` | Porcentaje actual de batería publicado por `battery_manager`. |
| `/nexo/battery/status` | `std_msgs/msg/String` | Estado operativo de la batería (`OK`, `LOW_BATTERY_RETURN`, `CRITICAL_CHARGE_NOW`). |

### Acciones de Nav2 Utilizadas
* `ComputePathToPose` (`getPath`): Cálculo previo de distancia real de ida y regreso.
* `NavigateToPose` (`goToPose`): Ejecución de trayectoria y monitoreo continuo de distancia restante.

---

## Comandos de Compilación y Ejecución

### 1. Requisitos previos activos
Para que `smac_navigator` opere correctamente, deben estar corriendo:
1. Stack de navegación con el mapa editado (`ros2 launch navigation navigation.launch.py map:=map_equipo3_alamcen_ed`), `scan_fixer.py` y `rf2o_scan_fixed.launch.py`[cite: 1].
2. Robot localizado en el mapa mediante *Publish 2D pose estimate* en Foxglove[cite: 1].
3. Nodo `battery_manager` activo para recibir telemetría energética en tiempo real.

### 2. Compilar el paquete
```zsh
source /opt/ros/humble/setup.zsh
cd ~/shared/nexo_ws
colcon build --packages-select nexo_mentorpi
source install/setup.zsh
export ROS_DOMAIN_ID=0
```

### 3. Ejecutar el navegador interactivo `smac_navigator`
```zsh
source /opt/ros/humble/setup.zsh
source ~/shared/nexo_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
ros2 run nexo_mentorpi smac_navigator
```

### 4. Uso en consola
1. Al iniciar, espera el mensaje `Nav2 listo para navegar!`.
2. Selecciona el número de la estación destino (`1` a `8`) y presiona `Enter`.
3. Revisa el análisis de distancia y energía requerida en pantalla; si el viaje es viable, el robot navegará automáticamente y mostrará la distancia restante en tiempo real.
4. Presiona `q` + `Enter` (o `Ctrl + C`) para salir y cancelar cualquier meta activa.

---
