# Navegación de dron con VFH y LiDAR

Este proyecto implementa una navegación sencilla para un dron usando sensores LiDAR y el algoritmo **VFH** (*Vector Field Histogram*) para evitar obstáculos mientras se dirige hacia un objetivo.

El programa despega el dron, lee datos del LiDAR, construye un histograma de obstáculos, calcula una dirección segura de movimiento y envía comandos de velocidad hasta llegar al punto objetivo. Al finalizar, el dron aterriza.

---

## Estructura del proyecto

```text
.
├── config.py              # Parámetros generales del sistema
├── mission.py             # Punto de entrada principal de la misión
├── drone_controller.py    # Control del dron y bucle de navegación
├── lidar_processor.py     # Lectura y filtrado de datos LiDAR
├── vfh_planner.py         # Implementación del algoritmo VFH
└── utils.py               # Funciones auxiliares
```

---

## Funcionamiento general

El flujo principal del sistema es el siguiente:

1. Se inicia ROS 2.
2. Se crea una instancia del dron con la configuración definida.
3. El dron se arma, entra en modo *offboard* y despega.
4. Se ejecuta la navegación hacia un objetivo `x, y, z`.
5. Durante la navegación:
   - Se obtiene la posición y orientación actual del dron.
   - Se leen las mediciones del LiDAR.
   - Se construye un histograma polar de obstáculos.
   - Se suaviza el histograma.
   - Se elige una dirección libre hacia la que moverse.
   - Se calcula la velocidad lineal y la velocidad de giro.
   - Se envían comandos de velocidad al dron.
6. Cuando el dron llega al objetivo, se detiene y aterriza.

---

## Módulos

### `mission.py`

Es el archivo principal que lanza la misión.

Sus responsabilidades son:

- Leer argumentos de línea de comandos.
- Inicializar ROS 2.
- Crear el objeto `VFHDrone`.
- Ejecutar el despegue.
- Llamar a `navigate_to_goal()`.
- Aterrizar el dron al terminar.
- Cerrar correctamente ROS 2.

Ejemplo de objetivo por defecto:

```python
[12.0, 0.0, 1.5]
```

Esto significa que el dron intentará ir a:

- `x = 12.0`
- `y = 0.0`
- `z = 1.5`

---

### `config.py`

Contiene el diccionario `DEFAULTS`, donde se agrupan los parámetros del sistema.

Algunos parámetros importantes son:

#### Parámetros VFH

| Parámetro | Descripción |
|---|---|
| `hist_bins` | Número de sectores del histograma polar. |
| `smooth_window` | Ventana usada para suavizar el histograma. |
| `threshold` | Umbral para decidir si un sector está libre u ocupado. |
| `influence_distance` | Distancia máxima a la que un obstáculo afecta a la navegación. |

#### Parámetros de sensores

| Parámetro | Descripción |
|---|---|
| `body_exclusion_radius` | Radio alrededor del dron que se ignora para evitar detectar el propio cuerpo. |
| `point_z_filter` | Filtrado vertical de puntos LiDAR. |
| `sensor_timeout` | Tiempo máximo permitido sin recibir datos recientes del sensor. |

#### Parámetros de control

| Parámetro | Descripción |
|---|---|
| `v_max` | Velocidad máxima horizontal. |
| `v_min` | Velocidad mínima horizontal. |
| `v_max_z` | Velocidad máxima vertical. |
| `yaw_rate_max` | Velocidad máxima de giro. |
| `min_height` | Altura mínima configurada para el dron. |

#### Parámetros de navegación

| Parámetro | Descripción |
|---|---|
| `goal_tol_xy` | Tolerancia horizontal para considerar alcanzado el objetivo. |
| `goal_tol_z` | Tolerancia vertical para considerar alcanzado el objetivo. |

---

### `drone_controller.py`

Contiene la clase principal `VFHDrone`, que hereda de `DroneInterfaceTeleop`.

Este módulo conecta el dron con el planificador VFH y el procesador LiDAR.

Sus partes principales son:

#### `VFHDrone`

Controla el ciclo de navegación.

Funciones importantes:

- `_goal_reached(pos, goal_xyz)`  
  Comprueba si el dron está suficientemente cerca del objetivo.

- `navigate_to_goal(goal_xyz, timeout_s=480.0, control_rate_hz=10.0)`  
  Ejecuta el bucle principal de navegación.

Dentro de `navigate_to_goal()` se realiza este proceso repetidamente:

1. Leer posición y orientación actual.
2. Calcular la dirección hacia el objetivo.
3. Obtener datos del LiDAR.
4. Construir el histograma de obstáculos.
5. Suavizar el histograma.
6. Seleccionar una dirección segura.
7. Calcular velocidad y giro.
8. Enviar comandos al dron.

Si el objetivo se alcanza, el dron hace `hover()` y la función devuelve `True`.

Si se supera el tiempo máximo o se detecta una situación sin salida, devuelve `False`.

#### `VFHDebugPlotter`

Clase opcional para visualizar el histograma VFH usando `matplotlib`.

Muestra:

- Histograma original.
- Histograma suavizado.
- Umbral de obstáculos.
- Dirección al objetivo.
- Dirección seleccionada.
- Orientación actual del dron.

Sirve para depurar y entender cómo el algoritmo decide la dirección de movimiento.

---

### `lidar_processor.py`

Se encarga de recibir y filtrar datos del LiDAR.

Este módulo se suscribe a dos tópicos:

```text
sensor_measurements/lidar/scan
sensor_measurements/lidar/points
```

Puede trabajar con:

- `LaserScan`
- `PointCloud2`

#### Funcionamiento

Para cada medición:

1. Comprueba que la distancia sea válida.
2. Ignora puntos demasiado cercanos al dron.
3. Ignora puntos más lejanos que `influence_distance`.
4. En el caso de nubes de puntos, filtra también por altura `z`.
5. Guarda las mediciones como pares:

```python
(angle, distance)
```

La función principal es:

```python
get_measurements(timeout)
```

Devuelve:

```python
measurements, source
```

Donde `source` puede ser:

- `"scan"`
- `"points"`
- `"none"`

Si los datos son demasiado antiguos, devuelve `None`.

---

### `vfh_planner.py`

Implementa el planificador **VFH**.

El objetivo de este módulo es elegir una dirección segura para avanzar evitando obstáculos.

#### Clase `VFHPlanner`

Funciones principales:

#### `build_histogram(position, yaw, measurements)`

Construye un histograma polar a partir de las mediciones del LiDAR.

Cada obstáculo aumenta el valor del sector angular correspondiente. Cuanto más cerca está el obstáculo, mayor es su influencia.

---

#### `smooth_histogram(hist)`

Suaviza el histograma para evitar decisiones bruscas causadas por ruido o mediciones aisladas.

---

#### `select_heading(yaw, goal_heading, histogram)`

Selecciona la dirección final de movimiento.

El proceso es:

1. Convierte la dirección del objetivo en un sector del histograma.
2. Comprueba si ese sector está libre.
3. Si está libre, avanza directamente hacia el objetivo.
4. Si está ocupado, busca valles libres en el histograma.
5. Escoge el valle más cercano al objetivo.
6. Calcula una dirección intermedia segura dentro del valle.
7. Si no hay ningún valle libre, activa `trap_detected`.

---

#### `compute_speed(...)`

Calcula:

- Velocidad horizontal.
- Velocidad de giro.

La velocidad se reduce cuando:

- Hay obstáculos en la dirección actual.
- El dron necesita girar mucho.

Esto permite que el dron avance más despacio en situaciones de riesgo.

---

### `utils.py`

Contiene funciones auxiliares simples.

#### `clamp(value, low, high)`

Limita un valor dentro de un rango.

Ejemplo:

```python
clamp(5, 0, 3)
# Resultado: 3
```

#### `wrap_pi(angle)`

Normaliza un ángulo al rango:

```text
[-pi, pi)
```

Esto es útil para comparar orientaciones y errores angulares.

---

## Ejecución

Ejemplo básico:

```bash
python3 mission.py
```

Ejemplo indicando el namespace del dron:

```bash
python3 mission.py -n drone0
```

Ejemplo indicando un objetivo personalizado:

```bash
python3 mission.py -n drone0 --goal 12.0 0.0 1.5
```

El argumento `--goal` recibe tres valores:

```bash
--goal X Y Z
```

Por ejemplo:

```bash
--goal 5.0 2.0 1.5
```

indica que el dron debe navegar hacia la posición:

```text
x = 5.0
y = 2.0
z = 1.5
```

---

## Dependencias principales

Este código utiliza:

- Python 3
- ROS 2
- `rclpy`
- `sensor_msgs`
- `sensor_msgs_py`
- `matplotlib`
- `as2_python_api`

También depende de una interfaz de dron compatible con:

```python
DroneInterfaceTeleop
```

---

## Tópicos LiDAR usados

El sistema espera recibir datos LiDAR desde estos tópicos:

```text
sensor_measurements/lidar/scan
sensor_measurements/lidar/points
```

El código prioriza los datos `LaserScan` si son recientes. Si no hay datos recientes de `LaserScan`, intenta usar datos `PointCloud2`.

---

## Parámetros importantes para ajustar

Si el dron evita obstáculos demasiado tarde, se puede aumentar:

```python
influence_distance
```

Si el dron considera demasiados sectores como ocupados, se puede subir:

```python
threshold
```

Si el dron se mueve demasiado rápido, se puede reducir:

```python
v_max
```

Si gira demasiado rápido, se puede reducir:

```python
yaw_rate_max
```

Si el histograma es muy ruidoso, se puede aumentar:

```python
smooth_window
```

---

## Salidas y comportamiento esperado

Durante la misión, el dron debería:

1. Despegar hasta la altura indicada.
2. Avanzar hacia el objetivo.
3. Evitar obstáculos dentro de la distancia de influencia.
4. Reducir la velocidad si detecta obstáculos cercanos.
5. Detenerse al llegar al objetivo.
6. Aterrizar.

Si no recibe datos LiDAR recientes, el dron se queda en `hover()` para evitar avanzar sin información del entorno.

Si el planificador no encuentra ningún camino libre, se considera una situación de bloqueo o `trap`, se detiene la navegación y el dron aterriza.