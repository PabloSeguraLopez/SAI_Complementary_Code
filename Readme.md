# Algoritmos de Navegación, Mapeo 3D y Planificación de Trayectorias

Este repositorio contiene código relacionado con robótica aérea y navegación autónoma, incluyendo:

- **Algoritmos de evitación de obstáculos**
- **Generación de mapas 3D**
- **Planificación de trayectorias**
- Integración con entornos de simulación basados en **ROS 2 Humble**, **Gazebo** y **Aerostack2**

El objetivo del repositorio es facilitar el desarrollo, prueba y validación de algoritmos de navegación en simulación, especialmente para plataformas aéreas no tripuladas.

---

## Requisitos

Usar la imagen de Docker:

```bash
aerostack2/humble
```

Esta imagen proporciona un entorno compatible con ROS 2 Humble y Aerostack2.

---

## Instalación

Una vez dentro del contenedor Docker basado en `aerostack2/humble`, ejecutar los siguientes comandos.

### 1. Actualizar el sistema e instalar dependencias de Gazebo

```bash
sudo apt update
sudo apt install -y ros-humble-as2-platform-gazebo ros-humble-as2-gazebo-assets
```

### 2. Clonar el proyecto de Gazebo de Aerostack2

```bash
cd /root
git clone https://github.com/aerostack2/project_gazebo
```

### 3. Aplicar corrección en la API de Python de Aerostack2

```bash
sed -i 's/@abstractmethod//g' /root/aerostack2_ws/build/as2_python_api/as2_python_api/modules/module_base.py
```

### 4. Clonar colecciones de modelos y mundos de Gazebo

```bash
git clone https://github.com/leonhartyao/gazebo_models_worlds_collection
git clone https://github.com/osrf/gazebo_models
```

### 5. Configurar la ruta de recursos de Gazebo

```bash
export GZ_SIM_RESOURCE_PATH=/root/gazebo_models_worlds_collection/worlds:/root/gazebo_models_worlds_collection/models:/root/gazebo_models:$GZ_SIM_RESOURCE_PATH
```

Para hacer esta configuración persistente, se puede añadir la línea anterior al archivo `~/.bashrc`:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=/root/gazebo_models_worlds_collection/worlds:/root/gazebo_models_worlds_collection/models:/root/gazebo_models:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 6. RTAB-Map

Para la generación del mapa 3D es necesario instalar RTAB-Map:

```bash
apt install ros-humble-rtabmap-ros
```

### 7. Matlab

Son necesarios "Computer Vision Toolbox" y "Image Processing Toolbox".
