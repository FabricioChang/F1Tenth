# Follow the Gap Controller para F1Tenth

## Autor

**Fabricio Chang**

Materia: Vehículos No Tripulados  
ESPOL

---

# Introducción

Este proyecto consiste en la implementación de un controlador reactivo para un vehículo F1Tenth utilizando ROS 2 Humble y el simulador oficial F1Tenth Gym.

El controlador fue desarrollado utilizando el algoritmo **Follow the Gap**, el cual permite navegar de forma autónoma utilizando únicamente la información obtenida por un sensor LiDAR, sin utilizar mapas, planificación global, SLAM o algoritmos de localización.

Como parte del proyecto también se implementó un sistema automático de:

- Conteo de vueltas.
- Cronómetro por vuelta.
- Registro del mejor tiempo de vuelta.
- Detención automática luego de completar 10 vueltas.

El controlador fue probado sobre el circuito **SaoPaulo**, logrando completar múltiples vueltas consecutivas sin colisiones.

---

# Algoritmo Follow the Gap

El algoritmo Follow the Gap es un método reactivo ampliamente utilizado para navegación autónoma.

Su funcionamiento consiste en identificar el mayor espacio libre disponible frente al vehículo y dirigir el automóvil hacia dicho espacio evitando obstáculos.

El algoritmo implementado sigue el siguiente flujo:

```
LaserScan

↓

Filtrado de datos

↓

Suavizado de lecturas

↓

Detección del obstáculo más cercano

↓

Creación de Bubble de seguridad

↓

Búsqueda del Largest Gap

↓

Selección del mejor punto del Gap

↓

Cálculo del ángulo de dirección

↓

Control adaptativo de velocidad

↓

Publicación de comandos Ackermann
```

Este enfoque permite navegar de forma robusta incluso en curvas cerradas sin necesidad de conocer previamente el mapa.

---

# Arquitectura del nodo

El controlador está implementado como un único nodo ROS 2.

## Subscribers

### /scan

Tipo:

```
sensor_msgs/msg/LaserScan
```

Recibe las mediciones del sensor LiDAR utilizadas para detectar obstáculos.

### /ego_racecar/odom

Tipo:

```
nav_msgs/msg/Odometry
```

Se utiliza para:

- obtener la posición del vehículo
- registrar el punto inicial
- contar las vueltas
- calcular el tiempo por vuelta

## Publisher

### /drive

Tipo:

```
ackermann_msgs/msg/AckermannDriveStamped
```

Publica:

- velocidad
- ángulo de dirección

para controlar el vehículo.

---

# Funcionamiento del controlador

## 1. Lectura del LiDAR

Cada vez que llega un mensaje del tópico **/scan**, el controlador obtiene todas las distancias medidas por el sensor.

Las lecturas inválidas son reemplazadas y posteriormente limitadas a un rango máximo de trabajo.

---

## 2. Suavizado

Las lecturas del LiDAR presentan pequeñas variaciones entre muestras consecutivas.

Para reducir el ruido se aplica un filtro de promedio móvil utilizando una ventana configurable.

Esto produce una trayectoria mucho más estable.

---

## 3. Bubble de seguridad

Se identifica el obstáculo más cercano.

Alrededor de dicho obstáculo se elimina una región denominada **Bubble**.

Esto evita que el vehículo intente conducir demasiado cerca de los obstáculos.

---

## 4. Largest Gap

Luego de eliminar la Bubble se busca el segmento continuo más grande libre de obstáculos.

Este segmento representa la mejor dirección posible para continuar avanzando.

---

## 5. Selección del mejor punto

No siempre es conveniente conducir exactamente hacia el punto más lejano.

Por esta razón el controlador calcula un punto intermedio entre:

- el centro del Largest Gap
- el punto más lejano dentro del Gap

Esto reduce las oscilaciones y mejora la estabilidad del vehículo.

---

## 6. Dirección

Una vez seleccionado el mejor punto del Largest Gap, se calcula el ángulo correspondiente dentro del LiDAR.

Ese ángulo es enviado como comando de dirección Ackermann.

El ángulo además se limita mediante un valor máximo para evitar maniobras demasiado agresivas.

---

## 7. Control de velocidad

La velocidad no es constante.

El controlador adapta automáticamente la velocidad considerando:

- el ángulo de dirección
- la distancia libre frente al vehículo

Cuando el vehículo circula en una recta y existe suficiente espacio libre, la velocidad aumenta.

Cuando detecta curvas o poco espacio disponible, la velocidad disminuye automáticamente para mantener la estabilidad.

---

# Conteo automático de vueltas

El proyecto implementa un contador automático de vueltas utilizando la odometría publicada por el simulador.

El procedimiento es el siguiente:

1. Se registra automáticamente la posición inicial del vehículo.
2. Cuando el vehículo se aleja una distancia mínima del punto inicial, el contador queda armado.
3. Cuando el vehículo vuelve a ingresar a una pequeña región alrededor del punto de inicio, se registra una nueva vuelta.
4. El contador vuelve a desarmarse hasta que el vehículo complete otra vuelta.

Este mecanismo evita contar varias veces la misma vuelta cuando el vehículo permanece cerca de la línea de salida.

---

# Cronómetro por vuelta

El controlador registra automáticamente:

- Tiempo de la vuelta actual
- Mejor tiempo registrado
- Tiempo total de ejecución

La información se muestra en la terminal durante toda la simulación.

Ejemplo:

```
Vuelta 1/10 completada
Tiempo vuelta: 49.39 s
Mejor vuelta: 49.39 s
Tiempo total: 49.39 s
```

---

# Parámetros utilizados

| Parámetro | Valor |
|-----------|------:|
| Bubble Radius | 13 |
| Ventana de suavizado | 5 |
| Ángulo frontal analizado | 100° |
| Máximo ángulo de dirección | 0.42 rad |
| Velocidad máxima | 8.2 m/s |
| Velocidad media | 4.8 m/s |
| Velocidad en curvas | 2.4 m/s |
| Distancia para rearmar contador | 8 m |
| Radio de detección de meta | 1.5 m |

---

# Estructura del código

El nodo está organizado en funciones independientes para facilitar su comprensión.

## odom_callback()

Se ejecuta cada vez que llega un mensaje de odometría.

Responsabilidades:

- registrar la posición inicial
- calcular distancia recorrida
- detectar nuevas vueltas
- calcular tiempos
- detener el vehículo luego de completar 10 vueltas

---

## scan_callback()

Función principal del algoritmo.

Responsabilidades:

- procesar el LiDAR
- eliminar ruido
- generar Bubble
- encontrar Largest Gap
- seleccionar el mejor punto
- calcular velocidad
- publicar comandos Ackermann

---

## smooth_ranges()

Aplica un filtro de promedio móvil sobre las mediciones del LiDAR.

---

## find_max_gap()

Busca el segmento continuo libre de obstáculos más grande.

---

## find_best_point()

Calcula el punto objetivo dentro del Largest Gap utilizando una combinación entre el centro del Gap y el punto más lejano.

---

## calculate_speed()

Determina la velocidad adecuada considerando:

- distancia libre al frente
- ángulo de dirección

---

## publish_drive()

Publica los comandos Ackermann hacia el tópico:

```
/drive
```

---

# Compilación

Desde la carpeta principal del workspace:

```bash
cd ~/F1Tenth-Repository

colcon build

source install/setup.bash
```

---

# Ejecución

## Terminal 1

Ejecutar el simulador.

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

## Terminal 2

Ejecutar el controlador.

```bash
source install/setup.bash

ros2 run f1tenth_gym_ros follow_the_gap
```

---

# Resultado esperado

Durante la ejecución el vehículo debe:

- recorrer el circuito de forma autónoma
- evitar colisiones
- completar 10 vueltas
- mostrar el tiempo de cada vuelta
- mostrar el mejor tiempo obtenido
- detenerse automáticamente al finalizar

---

# Resultados obtenidos

Mapa utilizado:

```
SaoPaulo
```

Resultados alcanzados:

- Controlador completamente reactivo basado en LiDAR.
- Conteo automático de vueltas mediante odometría.
- Cronómetro por vuelta implementado.
- Mejor tiempo de vuelta aproximado de **49 segundos**.
- Finalización automática luego de completar las 10 vueltas.

---

# Posibles mejoras

Aunque el controlador cumple correctamente con los objetivos planteados, existen varias mejoras que podrían implementarse en el futuro:

- Bubble Radius dinámico.
- Control PID para suavizar la dirección.
- Control adaptativo de velocidad basado en Time-To-Collision (TTC).
- Predicción del mejor Gap utilizando información temporal.
- Integración con algoritmos de planificación como Pure Pursuit o Stanley Controller.
- Uso de múltiples sensores para mejorar la robustez de la navegación.

---

# Conclusiones

El algoritmo Follow the Gap permitió desarrollar un controlador reactivo capaz de navegar de forma completamente autónoma utilizando únicamente la información del sensor LiDAR.

La incorporación de un control adaptativo de velocidad permitió aumentar el desempeño del vehículo en las rectas manteniendo una conducción estable en las curvas.

Además, la implementación del contador automático de vueltas y del cronómetro permitió evaluar objetivamente el rendimiento del controlador durante múltiples recorridos consecutivos.

En conjunto, el proyecto demuestra cómo un algoritmo reactivo relativamente sencillo puede resolver de forma eficiente el problema de navegación autónoma en un circuito cerrado, cumpliendo con todos los requerimientos establecidos para la competencia.
