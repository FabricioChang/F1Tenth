# Follow the Gap Controller para F1Tenth

## Autor

**Fabricio Chang**  
Materia: **Vehículos No Tripulados**
ESPOL
Link al video evidencia: https://youtu.be/pWDd3RFgN48
Se sugiere bajar el volumen del video luego del minuto 3
---

## 1. Introducción

Este repositorio contiene la implementación de un controlador reactivo para un vehículo autónomo tipo **F1Tenth**, desarrollado en **ROS 2 Humble** y ejecutado sobre el simulador **F1Tenth Gym ROS**.

El controlador utiliza el algoritmo **Follow the Gap**, un enfoque reactivo basado en LiDAR que permite que el vehículo navegue de forma autónoma evitando obstáculos, sin utilizar mapas globales, SLAM, planificación global ni rutas predefinidas.

Además del controlador principal, se implementaron mejoras para evaluar el rendimiento durante la simulación:

- Conteo automático de vueltas.
- Cronómetro por vuelta.
- Registro del mejor tiempo de vuelta.
- Tiempo total de ejecución.
- Detención automática al completar 10 vueltas.
- Control adaptativo de velocidad según la curva y la distancia libre al frente.

El controlador fue probado en el mapa **SaoPaulo**, logrando recorrer múltiples vueltas consecutivas con navegación autónoma basada únicamente en el sensor LiDAR y la odometría del vehículo.

---

## 2. Entorno recomendado

El proyecto debe ejecutarse en Linux, preferiblemente en una instalación nativa o máquina virtual con Ubuntu compatible con ROS 2 Humble.

| Componente | Versión recomendada |
|---|---|
| Sistema operativo | Ubuntu 22.04 LTS Jammy Jellyfish |
| ROS 2 | ROS 2 Humble Hawksbill |
| Lenguaje | Python 3.10 |
| Build system | colcon |
| Visualización | RViz2 |
| Simulador | F1Tenth Gym ROS |
| Tipo de controlador | Reactivo basado en LiDAR |

> Nota: ROS 2 Humble está diseñado para Ubuntu 22.04. Se recomienda no usar Ubuntu 24.04 para este proyecto si se desea evitar problemas de compatibilidad con paquetes de ROS 2 Humble.

---

## 3. Documentación oficial útil

Antes de ejecutar el proyecto, se recomienda revisar las siguientes documentaciones oficiales:

- Instalación oficial de ROS 2 Humble en Ubuntu 22.04:  
  https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

- Documentación general de ROS 2 Humble:  
  https://docs.ros.org/en/humble/index.html

- Tutorial oficial de RViz2 en ROS 2 Humble:  
  https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-Main.html

- Repositorio oficial de F1Tenth Gym ROS:  
  https://github.com/f1tenth/f1tenth_gym_ros

- Paquete `ackermann_msgs`:  
  https://index.ros.org/r/ackermann_msgs/

---

## 4. Instalación de dependencias

### 4.1. Actualizar Ubuntu

```bash
sudo apt update
sudo apt upgrade -y
```

### 4.2. Instalar ROS 2 Humble Desktop

La instalación recomendada es `ros-humble-desktop`, porque incluye herramientas comunes como RViz2 y paquetes básicos de simulación.

```bash
sudo apt install ros-humble-desktop -y
```

Si ROS 2 aún no está instalado, seguir primero la guía oficial completa:

```text
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
```

### 4.3. Configurar el entorno de ROS 2

Ejecutar en cada terminal nueva:

```bash
source /opt/ros/humble/setup.bash
```

Para no tener que repetirlo manualmente cada vez:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4.4. Instalar herramientas de compilación

```bash
sudo apt install python3-colcon-common-extensions python3-pip git -y
```

### 4.5. Instalar paquetes necesarios para F1Tenth

```bash
sudo apt install ros-humble-ackermann-msgs ros-humble-nav2-map-server ros-humble-rviz2 -y
```

Dependiendo de la instalación del simulador, también puede ser necesario instalar dependencias de Python:

```bash
pip3 install numpy scipy matplotlib gymnasium
```

---

## 5. Clonación y preparación del workspace

Ubicarse en la carpeta donde se trabajará el proyecto:

```bash
cd ~
```

Clonar el repositorio del proyecto o ingresar al repositorio ya existente:

```bash
cd ~/F1Tenth-Repository
```

Si se está instalando desde cero, el simulador base puede obtenerse desde el repositorio oficial:

```bash
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

Luego compilar el workspace:

```bash
cd ~/F1Tenth-Repository
colcon build
source install/setup.bash
```

Si se modifica el código Python del controlador y el paquete usa instalación con symlink, puede compilarse así:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 6. Descripción del enfoque utilizado

### 6.1. ¿Qué es Follow the Gap?

**Follow the Gap** es un algoritmo reactivo de navegación autónoma. Su objetivo es analizar las mediciones del LiDAR, encontrar el espacio libre más amplio frente al vehículo y dirigir el auto hacia una zona segura dentro de ese espacio.

A diferencia de algoritmos de planificación global, Follow the Gap no necesita conocer el mapa completo ni calcular una trayectoria desde el inicio hasta la meta. En cada instante toma una decisión local usando únicamente la información sensorial disponible.

### 6.2. Flujo general del algoritmo

```text
LaserScan
   ↓
Preprocesamiento de lecturas
   ↓
Limitación del rango máximo
   ↓
Suavizado del LiDAR
   ↓
Detección del obstáculo más cercano
   ↓
Creación de una burbuja de seguridad
   ↓
Búsqueda del largest gap
   ↓
Selección del mejor punto dentro del gap
   ↓
Cálculo del ángulo de dirección
   ↓
Control adaptativo de velocidad
   ↓
Publicación del comando Ackermann en /drive
```

---

## 7. Arquitectura ROS 2 del controlador

El controlador se implementó como un nodo ROS 2 en Python utilizando `rclpy`.

### 7.1. Nodo principal

```text
follow_the_gap
```

Este nodo concentra toda la lógica de navegación reactiva, control de velocidad, conteo de vueltas y registro de tiempos.

### 7.2. Subscribers

#### `/scan`

Tipo de mensaje:

```text
sensor_msgs/msg/LaserScan
```

Uso dentro del controlador:

- Obtener distancias del LiDAR.
- Detectar obstáculos cercanos.
- Buscar espacios libres.
- Calcular la dirección de avance.

#### `/odom` o `/ego_racecar/odom`

Tipo de mensaje:

```text
nav_msgs/msg/Odometry
```

Uso dentro del controlador:

- Leer la posición actual del vehículo.
- Registrar la posición inicial.
- Detectar el cruce por la zona de salida/meta.
- Contar vueltas completadas.
- Calcular tiempos por vuelta.

> En algunas configuraciones del simulador el tópico de odometría aparece como `/odom`, mientras que en otras puede aparecer como `/ego_racecar/odom`. Se recomienda verificarlo con:

```bash
ros2 topic list
```

### 7.3. Publisher

#### `/drive`

Tipo de mensaje:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

Uso dentro del controlador:

- Publicar velocidad.
- Publicar ángulo de dirección.
- Detener el vehículo al completar las 10 vueltas.

---

## 8. Funcionamiento interno del controlador

### 8.1. Lectura del LiDAR

El controlador recibe constantemente mensajes desde `/scan`. Cada mensaje contiene un arreglo de distancias medidas por el LiDAR.

Las lecturas inválidas, infinitas o no numéricas se reemplazan por valores seguros. Después, las distancias se limitan a un rango máximo de trabajo para evitar que valores demasiado grandes afecten la selección del gap.

---

### 8.2. Recorte del campo frontal

No se analiza todo el LiDAR completo, sino una región frontal del vehículo. Esto permite que el controlador se concentre en las zonas relevantes para avanzar.

El ángulo frontal usado fue aproximadamente:

```text
100 grados
```

Esto mejora la estabilidad porque evita que el vehículo tome decisiones basadas en obstáculos ubicados demasiado atrás o en zonas laterales poco relevantes.

---

### 8.3. Suavizado de lecturas

Para reducir ruido en el LiDAR se aplica un promedio móvil sobre las mediciones.

Parámetro usado:

```text
smoothing_window = 5
```

Este suavizado reduce oscilaciones en el volante y genera una trayectoria más estable.

---

### 8.4. Burbuja de seguridad

El controlador identifica el obstáculo más cercano dentro del campo analizado y crea una zona de exclusión alrededor de él.

Esta zona se conoce como **bubble**.

Parámetro usado:

```text
bubble_radius = 13
```

Todas las lecturas dentro de esa burbuja se eliminan temporalmente para evitar que el auto intente pasar demasiado cerca de una pared u obstáculo.

---

### 8.5. Búsqueda del largest gap

Después de eliminar la burbuja de seguridad, el algoritmo busca el segmento continuo más grande de lecturas válidas. Ese segmento representa el espacio libre más amplio disponible frente al vehículo.

El largest gap es importante porque evita seleccionar direcciones demasiado estrechas o peligrosas.

---

### 8.6. Selección del mejor punto

El controlador no apunta directamente al punto más lejano de forma agresiva. En su lugar, selecciona un punto balanceado dentro del gap, considerando:

- El centro del gap.
- El punto con mayor distancia libre.
- La necesidad de mantener una trayectoria suave.

Esto reduce zigzagueos y mejora la estabilidad en curvas.

---

### 8.7. Cálculo del ángulo de dirección

Una vez seleccionado el mejor punto, se convierte su índice dentro del arreglo LiDAR a un ángulo de dirección.

El ángulo se limita para evitar giros excesivos:

```text
max_steering = 0.42 rad
```

Este valor se publica como `steering_angle` dentro del mensaje Ackermann.

---

### 8.8. Control adaptativo de velocidad

La velocidad cambia automáticamente según dos factores principales:

- Qué tan cerrado es el giro.
- Cuánta distancia libre existe frente al vehículo.

Velocidades usadas:

| Situación | Velocidad aproximada |
|---|---:|
| Recta con espacio libre | 8.2 m/s |
| Curva media | 4.8 m/s |
| Curva cerrada o poco espacio | 2.4 m/s |

Esto permite avanzar rápido en rectas y reducir la velocidad en curvas para evitar colisiones.

---

## 9. Conteo automático de vueltas

El controlador utiliza la odometría para registrar vueltas automáticamente.

El procedimiento es:

1. Al iniciar, se guarda la posición inicial del vehículo.
2. El contador permanece desarmado mientras el vehículo está cerca de la salida.
3. Cuando el vehículo se aleja una distancia suficiente, el contador queda armado.
4. Cuando el vehículo vuelve a entrar en el radio de la posición inicial, se cuenta una vuelta.
5. El contador se vuelve a desarmar hasta que el vehículo se aleje nuevamente.

Parámetros usados:

| Parámetro | Valor |
|---|---:|
| Distancia para armar contador | 8.0 m |
| Radio de detección de meta | 1.5 m |
| Número total de vueltas | 10 |

Este mecanismo evita contar múltiples vueltas falsas si el vehículo permanece cerca de la línea de salida.

---

## 10. Cronómetro y mejor vuelta

El controlador mide automáticamente:

- Tiempo de la vuelta actual.
- Tiempo de cada vuelta completada.
- Mejor tiempo registrado.
- Tiempo total de ejecución.

Ejemplo de salida esperada en terminal:

```text
Vuelta 1/10 completada
Tiempo vuelta: 49.39 s
Mejor vuelta: 49.39 s
Tiempo total: 49.39 s
```

Al completar 10 vueltas, el controlador publica velocidad cero y detiene el vehículo.

---

## 11. Parámetros principales

| Parámetro | Valor |
|---|---:|
| Rango máximo del LiDAR | 10.0 m |
| Bubble Radius | 13 |
| Ventana de suavizado | 5 |
| Ángulo frontal analizado | 100° |
| Máximo ángulo de dirección | 0.42 rad |
| Velocidad máxima | 8.2 m/s |
| Velocidad media | 4.8 m/s |
| Velocidad en curvas | 2.4 m/s |
| Distancia para armar contador | 8.0 m |
| Radio de detección de meta | 1.5 m |
| Vueltas objetivo | 10 |

---

## 12. Estructura del código

El archivo principal del controlador contiene una clase que hereda de `Node` y organiza la lógica en funciones independientes.

### 12.1. `__init__()`

Inicializa el nodo y define:

- Subscribers.
- Publisher.
- Parámetros del LiDAR.
- Parámetros de velocidad.
- Variables para conteo de vueltas.
- Variables para cronómetro.

---

### 12.2. `odom_callback()`

Se ejecuta cada vez que llega un mensaje de odometría.

Responsabilidades:

- Registrar posición inicial.
- Calcular distancia respecto al punto de inicio.
- Armar o desarmar el contador de vueltas.
- Detectar vueltas completadas.
- Calcular tiempo de vuelta.
- Actualizar mejor vuelta.
- Detener el vehículo al completar 10 vueltas.

---

### 12.3. `scan_callback()`

Es la función principal del algoritmo Follow the Gap.

Responsabilidades:

- Leer datos del LiDAR.
- Filtrar lecturas inválidas.
- Recortar el campo frontal.
- Suavizar mediciones.
- Crear la burbuja de seguridad.
- Encontrar el largest gap.
- Seleccionar el mejor punto.
- Calcular dirección.
- Calcular velocidad.
- Publicar el comando Ackermann.

---

### 12.4. `smooth_ranges()`

Aplica un promedio móvil a las mediciones del LiDAR.

Objetivo:

- Reducir ruido.
- Evitar oscilaciones bruscas.
- Mejorar estabilidad del volante.

---

### 12.5. `find_max_gap()`

Busca el segmento continuo más grande de lecturas libres.

Objetivo:

- Identificar el espacio más seguro para avanzar.

---

### 12.6. `find_best_point()`

Selecciona el punto objetivo dentro del largest gap.

Objetivo:

- Balancear distancia libre y suavidad de trayectoria.

---

### 12.7. `calculate_speed()`

Calcula la velocidad según el ángulo de dirección y la distancia libre al frente.

Objetivo:

- Aumentar velocidad en rectas.
- Reducir velocidad en curvas.
- Evitar choques en zonas estrechas.

---

### 12.8. `publish_drive()`

Publica el mensaje `AckermannDriveStamped` en `/drive`.

Campos principales:

```text
msg.drive.speed
msg.drive.steering_angle
```

---

### 12.9. `stop_car()`

Publica un comando con velocidad cero para detener el vehículo.

Se ejecuta al completar las vueltas objetivo.

---

## 13. Compilación del proyecto

Desde la raíz del workspace:

```bash
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

Si hay errores por paquetes anteriores, se puede limpiar la compilación:

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

---

## 14. Ejecución del proyecto

Se recomienda usar al menos dos terminales.

### Terminal 1: ejecutar el simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Esto abre el simulador y RViz2.

---

### Terminal 2: ejecutar el controlador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run f1tenth_gym_ros follow_the_gap
```

---

## 15. Verificación de tópicos

Antes de ejecutar el controlador, se puede verificar que los tópicos necesarios estén activos:

```bash
ros2 topic list
```

Tópicos esperados:

```text
/scan
/odom
/drive
```

En algunas configuraciones puede aparecer:

```text
/ego_racecar/odom
```

Para inspeccionar el tipo de mensaje:

```bash
ros2 topic info /scan
ros2 topic info /drive
ros2 topic info /odom
```

Para observar la odometría:

```bash
ros2 topic echo /odom
```

Para observar los comandos enviados al vehículo:

```bash
ros2 topic echo /drive
```

---

## 16. Resultado esperado

Durante la simulación, el vehículo debe:

- Iniciar automáticamente al ejecutar el nodo.
- Leer el LiDAR desde `/scan`.
- Detectar el espacio libre más amplio.
- Girar hacia el mejor punto dentro del gap.
- Ajustar la velocidad según curvas y obstáculos.
- Completar vueltas en el circuito.
- Mostrar tiempos por vuelta en la terminal.
- Registrar la mejor vuelta.
- Detenerse automáticamente al completar 10 vueltas.

---

## 17. Resultados obtenidos

Mapa utilizado:

```text
SaoPaulo
```

Resultados alcanzados:

- Controlador reactivo implementado con Follow the Gap.
- Navegación basada únicamente en LiDAR para evitar obstáculos.
- Uso de odometría para evaluación de vueltas y tiempos.
- Conteo automático de vueltas completadas.
- Cronómetro por vuelta funcional.
- Mejor tiempo aproximado de vuelta: **49 segundos**.
- Detención automática al completar 10 vueltas.

---

## 18. Problemas comunes y soluciones

### 18.1. ROS 2 no reconoce los comandos

Ejecutar:

```bash
source /opt/ros/humble/setup.bash
```

O agregarlo al `.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### 18.2. No se encuentra el ejecutable `follow_the_gap`

Recompilar el workspace:

```bash
cd ~/F1Tenth-Repository
colcon build --symlink-install
source install/setup.bash
```

También verificar que el ejecutable esté declarado correctamente en el `setup.py` del paquete.

---

### 18.3. El vehículo no se mueve

Verificar que el controlador esté publicando en `/drive`:

```bash
ros2 topic echo /drive
```

Verificar que exista el tópico `/scan`:

```bash
ros2 topic list
```

---

### 18.4. No cuenta vueltas

Verificar el tópico correcto de odometría:

```bash
ros2 topic list | grep odom
```

Si el simulador usa `/ego_racecar/odom`, cambiar el subscriber del código o remapear el tópico.

---

### 18.5. RViz2 no abre correctamente

Verificar instalación:

```bash
sudo apt install ros-humble-rviz2 -y
```

Ejecutar RViz2 manualmente:

```bash
rviz2
```

---

## 19. Posibles mejoras futuras

- Bubble radius dinámico según la velocidad.
- Control PID para suavizar el ángulo de dirección.
- Control basado en Time-To-Collision.
- Predicción temporal del gap.
- Suavizado adicional del comando de dirección.
- Ajuste automático de velocidad según curvatura estimada.
- Integración con Pure Pursuit o Stanley Controller.
- Evaluación en más mapas además de SaoPaulo.

---

## 20. Conclusiones

El controlador desarrollado demuestra que un enfoque reactivo como **Follow the Gap** puede resolver de forma efectiva la navegación autónoma de un vehículo F1Tenth en un circuito cerrado.

El uso del LiDAR permite detectar obstáculos y seleccionar una dirección segura en tiempo real, mientras que la odometría permite registrar métricas de desempeño como vueltas completadas y tiempos por vuelta.

La incorporación de velocidad adaptativa fue fundamental para mejorar el rendimiento: el vehículo puede acelerar en rectas y reducir velocidad en curvas, disminuyendo el riesgo de colisión.

Finalmente, el contador automático de vueltas, el cronómetro y la detención al completar 10 vueltas permiten convertir el controlador en una solución completa para pruebas de competencia dentro del simulador F1Tenth Gym ROS.
