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


## Referencias

- Minguez, J. y Montano, L. (2004). *Nearness Diagram (ND) Navigation: Collision Avoidance in Troublesome Scenarios*. IEEE Transactions on Robotics and Automation, 20(1), 45–59.

- Labbé, M. y Michaud, F. (2019). *RTAB-Map as an Open-Source Lidar and Visual SLAM Library for Large-Scale and Long-Term Online Operation*. Journal of Field Robotics, 36(2), 416–446. https://doi.org/10.1002/rob.21831

- Erdogan, M. (2019). *Dataset of Gazebo Worlds, Models and Maps*. Repositorio de GitHub. https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps. Accedido el 9 de mayo de 2026.

- Fernandez-Cortizas, M., Molina, M., Arias-Perez, P., Perez-Segui, R., Perez-Saura, D. y Campoy, P. (2023). *Aerostack2: A software framework for developing multi-robot aerial systems*. arXiv preprint arXiv:2303.18237. https://doi.org/10.48550/arXiv.2303.18237

- Koenig, N. y Howard, A. (2004). *Design and Use Paradigms for Gazebo, An Open-Source Multi-Robot Simulator*. IEEE/RSJ International Conference on Intelligent Robots and Systems, 2149–2154. Sendai, Japón.

- Macenski, S., Foote, T., Gerkey, B., Lalancette, C. y Woodall, W. (2022). *Robot Operating System 2: Design, architecture, and uses in the wild*. Science Robotics, 7(66), eabm6074. https://doi.org/10.1126/scirobotics.abm6074

- Hart, P. E., Nilsson, N. J. y Raphael, B. (1968). *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*. IEEE Transactions on Systems Science and Cybernetics, 4(2), 100–107. https://doi.org/10.1109/TSSC.1968.300136

- Borenstein, J. y Koren, Y. (1991). *The Vector Field Histogram—Fast Obstacle Avoidance for Mobile Robots*. IEEE Transactions on Robotics and Automation, 7(3), 278–288. https://doi.org/10.1109/70.88137