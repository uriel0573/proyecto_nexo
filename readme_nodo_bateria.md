# Battery Manager Node (`nexo_mentorpi`)

Este repositorio contiene la implementación del nodo de gestión de batería real para el robot **MentorPi A1** (basado en tarjeta controladora STM32 / Yahboom) desarrollado en ROS 2 Humble.

## Descripción del Nodo

El nodo `battery_manager_node` se encarga de monitorear y procesar la telemetría eléctrica del robot en tiempo real para tomar decisiones de autonomía:

* **Suscripción de Hardware:** Escucha la telemetría del controlador base en el tópico `/ros_robot_controller/battery` (mensajes `std_msgs/msg/UInt16` en milivoltios).
* **Calibración y Filtro de Suavizado (EMA):** Aplica un filtro digital de promedio móvil exponencial (*Exponential Moving Average*) sobre el voltaje. Esto elimina las caídas repentinas de tensión (*voltage sag*) producidas por la demanda de corriente de los motores al acelerar o frenar.
* **Escala de Carga:** Mapea el rango operativo real de la batería LiPo ($7.0\text{V} = 0\%$ a $8.4\text{V} = 100\%$) a una escala porcentual continua ($0.0\%$ - $100.0\%$).
* **Publicación de Estados:** Envía el porcentaje calculado a `/nexo/battery/percentage` y el estado del sistema a `/nexo/battery/status` según los siguientes umbrales:
  * **`OK`**: Porcentaje superior al 30%.
  * **`LOW_BATTERY_RETURN`**: Porcentaje entre 20% y 30%.
  * **`CRITICAL_CHARGE_NOW`**: Porcentaje menor o igual al 20% (continúa reportando el porcentaje real de descarga continua sin forzar lecturas a 0).

---

## Instrucciones de Uso

### Prerrequisitos
Asegúrate de tener corriendo el driver del hardware en una terminal principal antes de ejecutar este nodo:

```zsh
source /opt/ros/humble/setup.zsh
source /home/ubuntu/ros2_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch bringup bringup.launch.py
```

---

### Compilación y Ejecución del Nodo

Abre una nueva terminal y sigue estos comandos para ingresar al workspace, compilar y ejecutar el nodo:

```zsh
# 1. Cargar el entorno de ROS 2 Humble
source /opt/ros/humble/setup.zsh

# 2. Navegar al workspace de desarrollo
cd ~/shared/nexo_ws

# 3. Compilar el paquete nexo_mentorpi
colcon build --packages-select nexo_mentorpi

# 4. Cargar el entorno del workspace compilado
source install/setup.zsh

# 5. Configurar el ID de dominio común para ROS 2
export ROS_DOMAIN_ID=0

# 6. Ejecutar el nodo de gestión de batería
ros2 run nexo_mentorpi battery_manager
```