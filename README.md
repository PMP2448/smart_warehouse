# smart_warehouse

## Descripción

**smart_warehouse** es un paquete ROS2 para la generación, monitoreo y gestión de marcadores ArUco en un almacén inteligente. Este proyecto incluye herramientas para:

- **Generación de marcadores ArUco**: Crear y exportar marcadores de códigos visuales para el posicionamiento de robots
- **Monitoreo de marcadores**: Detectar y rastrear marcadores en tiempo real a través de cámaras
- **Docking con ArUco**: Facilitar el acoplamiento automático de robots utilizando marcadores como referencia
- **Gestión de texturas y mundos de simulación**: Recursos para pruebas en Gazebo

## Requisitos

- **ROS2** (instalado y funcionando)
- **Python 3.8+**
- **Dependencias ROS2**: 
  - `rclpy`
  - `std_msgs`
  - `sensor_msgs`
  - `geometry_msgs`
  - `aruco_ros`
  - `tf2_ros`

## Instalación

### 1. Clonar el repositorio

Navega a la carpeta `src` de tu workspace de ROS2 y clona el repositorio:

```bash
cd ~/ros2_ws/src
git clone <url-del-repositorio> smart_warehouse
```

### 2. Compilar con colcon

Desde la raíz de tu workspace de ROS2, compila el paquete:

```bash
cd ~/ros2_ws
colcon build --packages-select smart_warehouse
```

Para compilar todo el workspace:

```bash
colcon build
```

### 3. Fuente del setup

Después de compilar, fuente el archivo de setup:

```bash
source install/setup.bash
```

## Uso

El paquete proporciona los siguientes nodos ejecutables:

- `aruco_generator`: Genera marcadores ArUco
- `monitor_aruco`: Monitorea y rastrea marcadores
- `docking_aruco`: Controla el acoplamiento de robots con marcadores
- `docking_aruco_v2`: Versión mejorada del control de acoplamiento

Para ejecutar un nodo:

```bash
ros2 run smart_warehouse <nombre-del-nodo>
```

## Estructura del proyecto

```
smart_warehouse/
├── aruco_generator.py       # Generador de marcadores ArUco
├── monitor_aruco.py         # Monitor de marcadores
├── docking_aruco.py         # Control de docking v1
├── docking_aruco_v2.py      # Control de docking v2
├── aruco_markers/           # Carpeta con marcadores generados
├── aruco_textures/          # Texturas para simulación
├── worlds/                  # Mundos de Gazebo
└── package.xml              # Configuración del paquete ROS2
```

## Licencia

Por definir

## Autor

pmp