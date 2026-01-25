# smart_warehouse

## Descripción

**smart_warehouse** es un paquete ROS2 que actualmente se centra en las pruebas y validación de marcadores ArUco para aplicaciones de robótica móvil. 

Aunque el proyecto forma parte de un desarrollo más amplio para un almacén automatizado, **en su estado actual este paquete contiene exclusivamente lo relativo a los tests de ArUco**, incluyendo herramientas para generación, detección, filtrado de pose y simulación de escenarios de prueba.

## Funcionalidades Actuales

- **Test de ArUco**: Entornos de simulación en MVSIM para validar la detección de marcadores.
- **Herramientas**:
  - Generación de marcadores (`aruco_generator`).
  - Monitoreo de detecciones (`monitor_aruco`).
  - Filtrado de pose (`aruco_pose_filter`) para mejorar la estabilidad de la detección.
  - Pruebas de docking/acoplamiento (`docking_aruco`).

## Requisitos

- **ROS2** (instalado y funcionando)
- **Python 3.8+**
- **Dependencias ROS2**: 
  - `rclpy`
  - `std_msgs`
  - `sensor_msgs`
  - `geometry_msgs`
  - `tf2_ros`
  - Paquete de detección de ArUco (ej. `ros2_aruco` o similar configurado en el launch)
  - `mvsim` (para la simulación)

## Instalación

### 1. Clonar el repositorio

Navega a la carpeta `src` de tu workspace de ROS2:

```bash
cd ~/ros2_ws/src
git clone <url-del-repositorio> smart_warehouse
```

### 2. Compilar con colcon

Desde la raíz de tu workspace de ROS2:

```bash
cd ~/ros2_ws
colcon build --packages-select smart_warehouse
```

### 3. Fuente del setup

```bash
source install/setup.bash
```

## Uso

### Lanzar entorno de pruebas

Para ejecutar la simulación de prueba de ArUco junto con la visualización en RViz:

```bash
ros2 launch smart_warehouse aruco_test.launch.py
```

### Nodos ejecutables

El paquete proporciona los siguientes nodos (ejecutables con `ros2 run smart_warehouse <nodo>`):

- `aruco_generator`: Genera imágenes de marcadores ArUco.
- `monitor_aruco`: Monitorea topics de ArUco.
- `aruco_pose_filter`: Nodo para filtrar y suavizar las poses detectadas.
- `docking_aruco`: Nodo para pruebas de lógica de acoplamiento.

## Estructura relevante

```
smart_warehouse/
├── aruco_markers/          # Marcadores generados
├── config/                 # Configuraciones (ej. filtros)
├── launch/                 # Launch files (aruco_test)
├── smart_warehouse/        # Código fuente Python
│   ├── aruco_generator.py
│   ├── aruco_pose_filter.py
│   ├── monitor_aruco.py
│   └── docking_aruco.py
├── rviz_configs/           # Configuraciones de visualización
├── worlds/                 # Mundos de prueba MVSIM
└── ...
```

## Autor

HJP Robotics
