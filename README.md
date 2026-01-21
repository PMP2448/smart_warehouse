# smart_warehouse

## Descripción

**smart_warehouse** es un paquete ROS2 para un almacén automatizado. Los robots siguen rutas predefinidas con Nav2 para el desplazamiento grueso y usan marcadores ArUco para la aproximación fina a la carga. El objetivo actual es validar el flujo completo en MVSIM y dejarlo listo para hardware real.

- **Navegación con Nav2**: Rutas predefinidas y planner de Nav2 para desplazamiento autónomo en el almacén.
- **Aproximación fina con ArUco**: Marcadores como referencia final antes de recoger la carga.
- **Herramientas ArUco**: Generación y detección de marcadores para pruebas y operación.
- **Simulación en MVSIM**: Mundos y recursos para probar sin hardware.
- **Mapas y rutas**: Mapas predefinidos y rutas almacenadas para configurar Nav2 según el layout del almacén.

**Estado**: Proyecto en desarrollo 🚀

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
├── aruco_markers/          # Marcadores generados
├── aruco_textures/         # Texturas para simulación
├── maps/                   # Mapas para Nav2 (layout del almacén)
├── routes/                 # Rutas predefinidas para Nav2
├── worlds/                 # Mundos MVSIM para simulación
├── smart_warehouse/        # Código fuente del paquete
│   ├── aruco_generator.py
│   ├── monitor_aruco.py
│   ├── docking_aruco.py
│   └── docking_aruco_v2.py
├── resource/               # Archivos de índice del paquete
├── test/                   # Tests y linters
├── package.xml             # Metadatos del paquete ROS2
├── setup.py                # Setup del paquete (ament_python)
├── setup.cfg               # Configuración de ament/flake8
└── other/                  # Notas u otros recursos
```

> Nota: los directorios `build/`, `install/` y `log/` los genera `colcon` y no se versionan.

## Licencia

Por definir

## Autor

HJP Robotics