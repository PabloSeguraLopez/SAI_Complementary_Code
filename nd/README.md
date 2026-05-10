# Navegación de dron con Nearness Diagram, LiDAR y métricas

Este proyecto implementa una navegación autónoma para un dron usando sensores LiDAR y una versión pragmática del algoritmo **Nearness Diagram (ND)** para evitar obstáculos mientras se dirige hacia un objetivo.

El programa despega el dron, lee datos del LiDAR, divide el entorno en sectores, detecta discontinuidades, construye valles navegables, selecciona una dirección segura de movimiento y envía comandos de velocidad hasta llegar al punto objetivo. Durante la misión también registra métricas, guarda la trayectoria y muestra gráficas en vivo.

---

## Estructura del proyecto

```text
.
├── nd_navigation.py       # Controlador principal de navegación ND
└── mission_metrics_*      # Carpetas generadas automáticamente con métricas de misión
    ├── summary.json       # Resumen de la misión
    ├── trajectory.csv     # Registro completo de trayectoria y decisiones
    └── trajectory_map.png # Imagen de la trayectoria seguida
```

El nombre `nd_navigation.py` es orientativo. Puede sustituirse por el nombre real del archivo Python usado en el repositorio.

---

## Funcionamiento general

El flujo principal del sistema es el siguiente:

1. Se inicia ROS 2.
2. Se crea una instancia del dron `PragmaticNDDrone`.
3. Se espera a recibir datos recientes del LiDAR.
4. El dron se arma.
5. Entra en modo *offboard*.
6. Despega hasta la altura configurada.
7. Se ejecuta la navegación hacia un objetivo `x, y, z`.
8. Durante la navegación:
   - Se obtiene la posición y orientación actual del dron.
   - Se leen las mediciones del LiDAR.
   - Se divide el entorno en sectores angulares.
   - Se calculan las distancias `pnd` y `rnd`.
   - Se detectan discontinuidades entre sectores.
   - Se construyen valles navegables.
   - Se selecciona el valle más adecuado.
   - Se clasifica la situación de navegación.
   - Se calcula el heading deseado.
   - Se calcula la velocidad lineal y la velocidad angular.
   - Se envían comandos de velocidad al dron.
   - Se actualizan métricas de misión.
   - Se actualizan las gráficas de depuración.
9. Cuando el dron llega al objetivo, se detiene y aterriza.
10. Al finalizar, se guardan las métricas de la misión.

---

## Módulos

### `nd_mission.py`

Es el archivo principal que lanza la misión y contiene todo el controlador de navegación.

Sus responsabilidades son:

- Inicializar ROS 2.
- Crear el objeto `PragmaticNDDrone`.
- Esperar datos del LiDAR.
- Armar el dron.
- Entrar en modo *offboard*.
- Despegar.
- Ejecutar la navegación.
- Enviar comandos de velocidad.
- Registrar métricas.
- Mostrar gráficas en vivo.
- Aterrizar el dron al terminar.
- Cerrar correctamente ROS 2.

La clase principal es:

```python
PragmaticNDDrone
```

Esta clase hereda de:

```python
DroneInterfaceTeleop
```

Ejemplo de objetivo configurado en el código:

```python
self.goal_xyz = [59.3, 87.7, 1.5]
```

Esto significa que el dron intentará ir a:

- `x = 59.3`
- `y = 87.7`
- `z = 1.5`

---

## Controlador `PragmaticNDDrone`

La clase `PragmaticNDDrone` implementa el controlador completo de navegación.

Sus partes principales son:

### Parámetros de misión

Define el objetivo del dron y la altura de despegue:

```python
self.goal_xyz = [59.3, 87.7, 1.5]
self.takeoff_height = 1.5
```

El código incluye varias metas comentadas para distintos escenarios:

```python
# Meta mapa de prueba columnas
# self.goal_xyz = [12.0, 0.0, 1.5]

# Escenario Fácil
# self.goal_xyz = [80.4, 34.3, 1.5]

# Escenario Medio
self.goal_xyz = [59.3, 87.7, 1.5]

# Escenario Difícil
# self.goal_xyz = [93.6, 74.1, 1.5]
```

Para cambiar el escenario basta con comentar o descomentar la meta deseada.

---

### Parámetros de plataforma

Definen las limitaciones físicas y de control del dron:

```python
self.robot_radius = 0.65
self.max_xy_speed = 1.00
self.max_z_speed = 0.12
self.max_yaw_rate = 0.70
```

También se configuran las tolerancias para considerar alcanzada la meta:

```python
self.goal_tol_xy = 1.0
self.goal_tol_z = 0.20
```

---

### Parámetros Nearness Diagram

El controlador divide el entorno en sectores angulares:

```python
self.n_sectors = 144
```

Cada sector representa una porción del espacio alrededor del dron.

Parámetros principales del algoritmo ND:

```python
self.max_range = 6.0
self.security_distance = 0.90
self.wide_valley_angle_deg = 70.0
self.front_valley_limit_deg = 110.0
self.max_heading_deg = 80.0
self.gap_local_window_deg = 30.0
self.discontinuity_threshold = 2.0 * self.robot_radius
self.min_valley_angle_deg = 12.0
```

Estos parámetros controlan:

- Alcance máximo considerado del LiDAR.
- Distancia de seguridad.
- Anchura mínima de valles.
- Región frontal válida.
- Límite máximo de heading lateral.
- Umbral para detectar discontinuidades.
- Ventana local alrededor del gap.

---

### Lectura y filtrado del LiDAR

El controlador se suscribe al tópico:

```text
sensor_measurements/lidar/scan
```

La función encargada de procesar el LiDAR es:

```python
scan_callback(self, msg: LaserScan)
```

Para cada medición:

1. Comprueba que la distancia sea válida.
2. Limita la distancia al rango máximo permitido.
3. Convierte la lectura polar a coordenadas `x, y` en el cuerpo del dron.
4. Ignora puntos demasiado cercanos al dron.
5. Asigna cada punto válido a un sector angular.
6. Guarda la distancia mínima de cada sector.
7. Calcula `pnd` y `rnd`.

El filtro de retornos propios se controla con:

```python
self.self_filter_radius = 0.35
```

Los puntos por debajo de este radio se ignoran para evitar detectar el propio cuerpo del dron como obstáculo.

---

## Algoritmo Nearness Diagram

El algoritmo utiliza dos representaciones principales:

```python
self.pnd
self.rnd
```

Donde:

- `pnd`: distancia mínima detectada por sector.
- `rnd`: distancia reducida, descontando el radio del dron.

```python
self.rnd = [max(0.0, d - self.robot_radius) for d in self.pnd]
```

---

### Detección de discontinuidades

La función:

```python
detect_discontinuities()
```

compara sectores consecutivos:

```python
delta = self.pnd[j] - self.pnd[i]
```

Si la diferencia supera el umbral:

```python
self.discontinuity_threshold
```

se registra una discontinuidad.

Cada discontinuidad guarda:

- Sector izquierdo.
- Sector derecho.
- Diferencia de distancia.
- Si la distancia aumenta de izquierda a derecha.

---

### Construcción de valles

La función:

```python
build_valleys(discs)
```

usa las discontinuidades para construir posibles valles navegables.

Cada valle se representa mediante:

```python
Valley
```

y contiene:

- Sector inicial.
- Sector final.
- Discontinuidad izquierda.
- Discontinuidad derecha.

---

### Selección de valle

La función:

```python
select_valley(valleys, s_goal, goal_heading_body)
```

elige el valle más adecuado.

Para seleccionar el valle se tiene en cuenta:

- Que el valle esté en la zona frontal.
- Que sea navegable.
- Que tenga suficiente anchura.
- Que tenga suficiente distancia libre.
- Que esté cerca de la dirección de la meta.
- Que no implique cambios bruscos respecto al valle anterior.

La penalización por cambio de valle se controla con:

```python
self.valley_switch_penalty = 10.0
```

---

### Clasificación de situaciones

La función principal de decisión es:

```python
classify_and_decide()
```

Esta función clasifica la navegación en una de varias situaciones.

#### `HSGR`

La meta está dentro del valle seleccionado o se puede avanzar hacia ella directamente.

```text
HSGR = High Safety Goal Region
```

#### `HSWR`

El valle es ancho, pero la meta no está directamente dentro de él.

```text
HSWR = High Safety Wide Region
```

En este caso el dron realiza un bordeo controlado.

#### `HSNR`

El valle es estrecho.

```text
HSNR = High Safety Narrow Region
```

En este caso el dron avanza hacia el centro del valle.

#### `LS1`

Hay un obstáculo peligroso a un lado del gap local.

```text
LS1 = Low Safety 1 obstacle
```

#### `LS2`

Hay obstáculos peligrosos a ambos lados del gap local.

```text
LS2 = Low Safety 2 obstacles
```

#### `SAFETY`

Se activa cuando un obstáculo está demasiado cerca:

```python
self.too_close_override_dist = 1.35
```

En esta situación el dron intenta alejarse del obstáculo antes de volver al comportamiento ND normal.

#### `ESCAPE`

Se activa cuando el detector de atasco considera que el dron no está progresando.

#### `GOAL`

Indica que la meta ha sido alcanzada.

---

## Cálculo de velocidades

La función:

```python
compute_command()
```

calcula el comando final que se envía al dron.

Devuelve:

```python
vx, vy, vz, yaw_rate, goal_reached, decision
```

Donde:

- `vx`: velocidad en X.
- `vy`: velocidad en Y.
- `vz`: velocidad vertical.
- `yaw_rate`: velocidad angular.
- `goal_reached`: indica si la meta fue alcanzada.
- `decision`: decisión ND generada.

---

### Velocidad lineal

La función:

```python
translational_speed(situation, desired_heading_body, goal_dist_xy)
```

ajusta la velocidad horizontal según:

- La situación ND actual.
- La distancia al obstáculo más cercano.
- La distancia a la meta.
- El heading deseado.

La velocidad se reduce cuando:

- El dron está cerca de obstáculos.
- El dron se acerca a la meta.
- El heading deseado está muy desalineado.
- La situación es `LS1`, `LS2`, `HSNR`, `HSWR` o `SAFETY`.

---

### Velocidad vertical

La velocidad vertical se calcula a partir del error en altura:

```python
vz = clamp(self.k_z * dz, -self.max_z_speed, self.max_z_speed)
```

Si el error vertical es pequeño, se manda:

```python
vz = 0.0
```

---

### Velocidad angular

La velocidad angular se calcula con:

```python
yaw_rate = clamp(self.k_yaw * yaw_error, -yaw_limit, yaw_limit)
```

El límite de giro depende de la situación:

- En `LS1` y `LS2` se limita el giro.
- En `SAFETY` se limita aún más.
- En navegación normal se usa `self.max_yaw_rate`.

---

## Histéresis y suavizado

El controlador usa histéresis para evitar cambios rápidos de situación:

```python
self.situation_hold_time = 0.8
```

También suaviza el heading deseado:

```python
self.max_heading_step_deg = 18.0
```

Esto reduce oscilaciones y cambios bruscos de dirección.

---

## Detección de atasco

El controlador incluye un detector de atasco basado en una ventana temporal de posiciones:

```python
self.stuck_window_sec = 2.5
self.stuck_min_progress = 0.05
self.stuck_min_goal_progress = 0.20
self.stuck_min_samples = 6
```

Durante la navegación se guarda un historial de:

```python
tiempo, x, y, distancia_a_meta
```

Si el dron apenas se mueve y no reduce la distancia a la meta, se considera atascado.

Cuando se detecta atasco:

1. Se activa el modo `ESCAPE`.
2. Se manda una maniobra de retroceso.
3. Se realiza un giro.
4. Se ignora temporalmente el detector para evitar reactivaciones inmediatas.

---

## Métricas

El sistema registra métricas durante toda la misión.

Al iniciar la navegación se crea una carpeta:

```text
mission_metrics_<fecha>_nd_10
```

Por ejemplo:

```text
mission_metrics_20260510_153012_nd_10
```

Dentro de esta carpeta se generan:

```text
summary.json
trajectory.csv
trajectory_map.png
```

---

### `summary.json`

Contiene un resumen de la misión:

- Si la misión terminó con éxito.
- Razón de finalización.
- Meta usada.
- Posición final.
- Error final 3D.
- Error final XY.
- Error final en Z.
- Distancia mínima alcanzada a la meta.
- Número de posibles choques.
- Número de posibles atascos.
- Tiempo total.
- Número de muestras.
- Conteo de situaciones ND.

---

### `trajectory.csv`

Contiene una fila por iteración del bucle de navegación.

Incluye:

- Tiempo.
- Posición `x, y, z`.
- Orientación `roll, pitch, yaw`.
- Posición de la meta.
- Distancia a la meta.
- Error vertical.
- Error 3D.
- Distancias mínimas a obstáculos.
- Situación ND.
- Razón de la decisión.
- Sectores relevantes.
- Información del valle seleccionado.
- Comandos enviados.
- Heading comandado.
- Heading hacia la meta.
- Eventos de posible choque.
- Eventos de posible atasco.

---

### `trajectory_map.png`

Guarda una imagen de la trayectoria en el plano XY.

La imagen muestra:

- Inicio.
- Meta.
- Posición final.
- Trayectoria seguida.
- Posibles choques.
- Posibles atascos.

---

## Visualización en vivo

El código incluye una visualización en vivo con `matplotlib`.

La ventana se divide en cuatro partes:

```text
┌─────────────────────────┬─────────────────────────┐
│                         │ Altura / distancias      │
│                         ├─────────────────────────┤
│ Radar                   │ Velocidades lineales     │
│                         ├─────────────────────────┤
│                         │ Velocidad angular        │
└─────────────────────────┴─────────────────────────┘
```

---

### Radar

El radar ocupa toda la parte izquierda.

Muestra:

- Dron.
- Obstáculos detectados.
- Puntos ignorados por el filtro propio.
- Dirección de la meta.
- Heading comandado.
- Sector `s_theta`.
- Distancia de seguridad.
- Distancia de caution.
- Radio de filtro propio.
- Zona de ralentización cerca de la meta.

---

### Altura, meta y distancias

La gráfica superior derecha muestra valores en metros:

- Altura actual `z`.
- Altura objetivo.
- Distancia XY a la meta.
- Distancia frontal mínima.
- Distancia izquierda mínima.
- Distancia derecha mínima.
- Distancia global mínima.

---

### Velocidades lineales

La gráfica central derecha muestra velocidades en m/s:

- `|vxy|`
- `vx`
- `vy`
- `vz`

---

### Velocidad angular

La gráfica inferior derecha muestra la velocidad angular en grados por segundo:

```text
yaw_rate
```

La velocidad angular se representa en una gráfica separada para no mezclar escalas de `m/s` y `deg/s`.

---

## Ejecución

Ejemplo básico:

```bash
bash launch_as2.bash
```

```bash
python3 nd_navigation.py
```

O usando el script de bash incluido `run_both.bash`

```bash
bash run_both.bash
```

**Importante:** Se debe modificar la variable `PYTHON_SCRIPT` con el nombre del script de Python a ejecutar.

---

## Dependencias principales

Este código utiliza:

- Python 3
- ROS 2
- `rclpy`
- `sensor_msgs`
- `matplotlib`
- `as2_python_api`
- Aerostack2
- Gazebo

También depende de una interfaz de dron compatible con:

```python
DroneInterfaceTeleop
```

---

## Tópicos LiDAR usados

El sistema espera recibir datos LiDAR desde:

```text
sensor_measurements/lidar/scan
```

El mensaje esperado es:

```python
sensor_msgs.msg.LaserScan
```

Si no se reciben datos recientes del LiDAR, el dron se queda en `hover()` para evitar avanzar sin información del entorno.

El tiempo máximo permitido sin LiDAR reciente es:

```python
self.scan_timeout = 0.8
```

---

## Parámetros importantes para ajustar

Si el dron se acerca demasiado a obstáculos, se puede aumentar:

```python
self.security_distance
self.too_close_override_dist
```

Si el dron va demasiado lento, se puede aumentar:

```python
self.max_xy_speed
```

Si el dron gira demasiado rápido, se puede reducir:

```python
self.max_yaw_rate
self.k_yaw
```

Si el dron oscila demasiado al elegir valles, se puede aumentar:

```python
self.valley_switch_penalty
self.situation_hold_time
```

Si el dron tarda demasiado en reaccionar a cambios de heading, se puede aumentar:

```python
self.max_heading_step_deg
```

Si el dron se queda atascado con facilidad, se pueden ajustar:

```python
self.stuck_window_sec
self.stuck_min_progress
self.stuck_min_goal_progress
self.post_escape_ignore_until
```

Si se detectan falsos obstáculos del propio dron, se puede aumentar:

```python
self.self_filter_radius
```

---

## Salidas y comportamiento esperado

Durante la misión, el dron debería:

1. Esperar a recibir LiDAR.
2. Armarse.
3. Entrar en modo *offboard*.
4. Despegar hasta la altura configurada.
5. Avanzar hacia el objetivo.
6. Detectar obstáculos con LiDAR.
7. Seleccionar valles navegables.
8. Evitar obstáculos cercanos.
9. Reducir la velocidad en zonas estrechas o peligrosas.
10. Activar modo `SAFETY` si un obstáculo está demasiado cerca.
11. Activar modo `ESCAPE` si se detecta atasco.
12. Registrar métricas de misión.
13. Mostrar gráficas en vivo.
14. Detenerse al llegar al objetivo.
15. Aterrizar.

Si no recibe datos LiDAR recientes, el dron se queda en `hover()`.

Si se supera el tiempo máximo de navegación, se detiene la misión y se guardan las métricas con razón de finalización:

```text
timeout
```

Si se produce un fallo enviando comandos de velocidad, la misión termina con:

```text
speed_command_error
```

Si ROS 2 deja de estar activo, la misión termina con:

```text
rclpy_not_ok
```