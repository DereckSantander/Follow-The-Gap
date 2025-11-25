# F1Tenth Follow the Gap - Controlador Reactivo Autónomo

Este proyecto implementa un controlador reactivo para carreras autónomas F1Tenth utilizando el algoritmo **Follow the Gap**, con sistema integrado de conteo de vueltas y cronometraje.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![Python](https://img.shields.io/badge/Python-3.8+-green)

---

## 📋 Tabla de Contenidos

1. [Descripción del Enfoque](#-descripción-del-enfoque)
2. [Características](#-características)
3. [Requisitos](#-requisitos)
4. [Estructura del Código](#-estructura-del-código)
5. [Instalación](#-instalación)
6. [Instrucciones de Ejecución](#-instrucciones-de-ejecución)
7. [Configuración de Parámetros](#-configuración-de-parámetros)
8. [Funcionamiento del Sistema](#-funcionamiento-del-sistema)
9. [Resultados](#-resultados)

---

## 🎯 Descripción del Enfoque

### ¿Qué es Follow the Gap?

**Follow the Gap** es un algoritmo de navegación reactiva que permite a un vehículo autónomo navegar de forma segura en entornos dinámicos utilizando únicamente datos de LiDAR. El algoritmo se basa en tres principios fundamentales:

1. **Seguridad**: Crear una zona de seguridad ("burbuja") alrededor de obstáculos cercanos
2. **Oportunidad**: Identificar el espacio libre más grande disponible
3. **Acción**: Dirigirse hacia el centro de ese espacio libre

### Funcionamiento del Algoritmo

```
┌─────────────────────────────────────────────────────────┐
│                    DATOS DE LIDAR                       │
│                          ↓                              │
│              1. Preprocesamiento                        │
│         (Limpieza de valores infinitos)                 │
│                          ↓                              │
│              2. Creación de Burbuja                     │
│      (Enmascarar zona alrededor del obstáculo           │
│               más cercano)                              │
│                          ↓                              │
│          3. Detección del Gap Máximo                    │
│      (Encontrar el espacio libre más grande)            │
│                          ↓                              │
│         4. Selección del Punto Objetivo                 │
│           (Centro del gap identificado)                 │
│                          ↓                              │
│          5. Control de Velocidad Adaptativo             │
│      (Ajustar velocidad según curvatura)                │
│                          ↓                              │
│              6. Suavizado de Control                    │
│         (Filtro para estabilizar dirección)             │
│                          ↓                              │
│                COMANDO DE CONTROL                       │
│            (velocidad + ángulo de giro)                 │
└─────────────────────────────────────────────────────────┘
```

### Ventajas del Enfoque

- ✅ **Reactivo y rápido**: No requiere mapeo ni planificación previa
- ✅ **Robusto**: Maneja entornos dinámicos y desconocidos
- ✅ **Eficiente**: Bajo costo computacional
- ✅ **Adaptativo**: Ajusta velocidad según la situación

---

## ✨ Características

- 🏎️ **Navegación autónoma** con algoritmo Follow the Gap
- 📊 **Conteo automático de vueltas** (basado en odometría)
- ⏱️ **Cronometraje por vuelta** con registro de tiempos
- 🎯 **Control de velocidad adaptativo** según curvatura
- 🛡️ **Sistema de evasión de emergencia** para obstáculos muy cercanos
- 🎨 **Visualización en tiempo real** con logs coloridos
- 🏁 **Detección automática de fin de carrera** (10 vueltas)
- 📈 **Resumen de carrera** con todos los tiempos y estadísticas

---

## 🔧 Requisitos

### Software

- **ROS2 Humble** (o superior)
- **Python 3.8+**
- **Simulador F1Tenth**

### Dependencias ROS2

```bash
sudo apt install ros-humble-ackermann-msgs
sudo apt install ros-humble-nav-msgs
```

### Dependencias Python

```bash
pip install numpy
```

---

## 📁 Estructura del Código

```
f1tenth_follow_gap/
├── f1tenth_follow_gap/
│   ├── __init__.py
│   └── follow_gap_node.py          # Nodo principal del algoritmo
├── resource/
│   └── f1tenth_follow_gap
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                     # Manifiesto del paquete ROS2
├── setup.py                        # Configuración del paquete Python
├── setup.cfg
└── README.md                       # Este archivo
```

### Componentes Principales del Código

#### 1. **Clase `FollowGapNode`**
Nodo principal ROS2 que implementa toda la lógica del controlador.

**Métodos principales:**

| Método | Descripción |
|--------|-------------|
| `__init__()` | Inicializa el nodo, parámetros, publishers y subscribers |
| `odom_callback()` | Procesa datos de odometría y establece línea de meta |
| `lidar_callback()` | Callback principal que ejecuta el algoritmo Follow the Gap |
| `preprocess_lidar()` | Limpia y preprocesa datos del LiDAR |
| `find_max_gap()` | Identifica el gap (espacio libre) más grande |
| `find_best_point()` | Calcula el punto objetivo dentro del gap |
| `_detect_lap_completion()` | Detecta cruce de línea de meta para conteo de vueltas |
| `_display_lap_message()` | Muestra mensaje formateado al completar vuelta |
| `_display_vehicle_status()` | Visualiza estado del vehículo en tiempo real |

#### 2. **Sistema de Estados**

El controlador mantiene diversos estados para la operación:

```python
# Estados de carrera
self.lap_count              # Número de vueltas completadas
self.lap_times              # Lista con tiempos de cada vuelta
self.race_finished          # Bandera de fin de carrera

# Estados de control
self.mode                   # Modo actual (STRAIGHT, TURN, EMERGENCY, etc.)
self.last_speed             # Velocidad actual
self.prev_angle             # Ángulo anterior (para suavizado)

# Estados de detección de vueltas
self.crossed_halfway        # Indica que salió de zona de inicio
self.in_finish_zone         # Indica que está en zona de meta
```

#### 3. **Parámetros Configurables**

Todos los parámetros son ajustables vía ROS2 parameters:

```python
max_speed               # Velocidad máxima (m/s)
min_speed               # Velocidad mínima (m/s)
bubble_radius           # Radio de la burbuja de seguridad (m)
gap_threshold           # Umbral para considerar espacio como gap (m)
smoothing_alpha         # Factor de suavizado (0-1)
max_steering_rate       # Tasa máxima de cambio de dirección (rad/s)
```

---

## 🚀 Instalación

### 1. Clonar o crear el workspace

```bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
```

### 2. Copiar el paquete

Copiar la carpeta `f1tenth_follow_gap` al directorio `src` de tu workspace.

### 3. Instalar dependencias

```bash
cd ~/f1tenth_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Compilar el paquete

```bash
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_follow_gap
```

### 5. Configurar el entorno

```bash
source ~/f1tenth_ws/install/setup.bash
```

**Tip**: Agregar esta línea al `~/.bashrc` para que se ejecute automáticamente:

```bash
echo "source ~/f1tenth_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🎮 Instrucciones de Ejecución

### Ejecución Básica

#### 1. Lanzar el simulador F1Tenth

En una terminal:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

#### 2. Ejecutar el controlador Follow the Gap

En otra terminal:

```bash
ros2 run f1tenth_follow_gap follow_gap_node
```

### Ejecución con Parámetros Personalizados

Puedes modificar los parámetros en tiempo de ejecución:

```bash
ros2 run f1tenth_follow_gap follow_gap_node \
    --ros-args \
    -p max_speed:=6.0 \
    -p min_speed:=2.5 \
    -p bubble_radius:=0.8
```

### Verificar Topics

Para verificar que el nodo está publicando correctamente:

```bash
# Ver topics activos
ros2 topic list

# Monitorear comandos de control
ros2 topic echo /drive

# Monitorear odometría
ros2 topic echo /ego_racecar/odom
```

### Verificar Nodos

```bash
# Listar nodos activos
ros2 node list

# Información del nodo
ros2 node info /follow_gap_node
```

---

## ⚙️ Configuración de Parámetros

### Parámetros de Velocidad

```bash
max_speed: 7.0          # Velocidad máxima en rectas (m/s)
min_speed: 2.8          # Velocidad mínima en curvas cerradas (m/s)
```

**Recomendaciones:**
- Aumentar `max_speed` para pistas amplias y rectas largas
- Reducir `min_speed` si el vehículo derrapa en curvas cerradas

### Parámetros del Algoritmo

```bash
bubble_radius: 0.7      # Radio de la zona de seguridad (metros)
gap_threshold: 1.5      # Distancia mínima para considerar gap válido (metros)
```

**Recomendaciones:**
- Aumentar `bubble_radius` en entornos con obstáculos muy densos
- Reducir `gap_threshold` si el algoritmo no encuentra gaps en espacios estrechos

### Parámetros de Control

```bash
smoothing_alpha: 0.85       # Factor de suavizado (0=suave, 1=reactivo)
max_steering_rate: 5.0      # Velocidad máxima de giro (rad/s)
angle_deadband: 0.0         # Zona muerta de ángulo (radianes)
```

**Recomendaciones:**
- Aumentar `smoothing_alpha` para comportamiento más reactivo
- Reducir `max_steering_rate` si el vehículo oscila demasiado

### Parámetros de Detección de Vueltas

```bash
lap_detection_threshold: 1.0    # Distancia para detectar línea de meta (m)
halfway_distance: 2.5           # Distancia para considerar salida de zona (m)
```

---

## 🔄 Funcionamiento del Sistema

### 1. Inicialización

Al ejecutar el nodo:

1. Se inicializan todos los parámetros y variables de estado
2. Se crean los publishers y subscribers
3. Se espera la primera lectura de odometría para establecer la línea de meta

```
📍 Línea de meta establecida en: x=0.00, y=0.00
🔄 Sistema de vueltas inicializado
```

### 2. Bucle Principal (LiDAR Callback)

En cada recepción de datos LiDAR (~40 Hz):

```python
1. Preprocesar datos LiDAR
2. Crear burbuja alrededor del obstáculo más cercano
3. Encontrar el gap más grande
4. Calcular ángulo hacia el centro del gap
5. Determinar velocidad según curvatura
6. Aplicar suavizado al ángulo
7. Publicar comando de control
8. Detectar cruce de línea de meta
9. Mostrar estado del vehículo
```

### 3. Sistema de Modos

El controlador cambia dinámicamente entre varios modos:

| Modo | Condición | Velocidad | Descripción |
|------|-----------|-----------|-------------|
| `EMERGENCY` | Obstáculo < 0.3m | 30% mínima | Evasión de emergencia |
| `SLOW` | Distancia < 0.8m | 80% mínima | Aproximación cuidadosa |
| `SHARP_TURN` | Ángulo > 0.4 rad | 120% mínima | Curva cerrada |
| `TURN` | Ángulo > 0.25 rad | 50% rango | Curva media |
| `CURVE` | Ángulo > 0.12 rad | 75% rango | Curva suave |
| `STRAIGHT` | Ángulo < 0.12 rad | Máxima | Recta |
| `FINISHED` | 10 vueltas | 0 m/s | Carrera finalizada |

### 4. Detección de Vueltas

Sistema de estados para contar vueltas correctamente:

```
┌──────────────────────────────────────────────────────────┐
│  INICIO: Robot en línea de meta                         │
│  distance_to_start = 0                                   │
└────────────────────┬─────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────────────────┐
│  SALIENDO: Robot sale de zona de inicio                 │
│  distance_to_start > halfway_distance (2.5m)            │
│  crossed_halfway = True                                  │
└────────────────────┬─────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────────────────┐
│  DANDO VUELTA: Robot circula por la pista               │
│  crossed_halfway = True                                  │
│  in_finish_zone = False                                  │
└────────────────────┬─────────────────────────────────────┘
                     ↓
┌──────────────────────────────────────────────────────────┐
│  REGRESANDO: Robot vuelve a línea de meta               │
│  distance_to_start < lap_detection_threshold (1.0m)     │
│  ⚡ VUELTA CONTADA                                      │
│  lap_count++, lap_time registrado                        │
│  crossed_halfway = False                                 │
│  in_finish_zone = True                                   │
└────────────────────┬─────────────────────────────────────┘
                     ↓
                  REPETIR
```

### 5. Visualización en Tiempo Real

Durante la carrera, se muestra información en tiempo real:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
📊 ESTADO DEL VEHÍCULO
  🔄 Vueltas:             3/10
  ⏱️  Última vuelta:        12.45s
  📍 Posición:            x= +2.34m, y= -1.56m
  🚗 Velocidad:           5.80 m/s
  🎯 Modo:                CURVE
  🎮 Ángulo:              +0.234 rad (raw: +0.267)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 6. Finalización

Al completar 10 vueltas, se muestra el resumen completo:

```
======================================================================
🏁🏁🏁 ¡CARRERA FINALIZADA! 🏁🏁🏁
   10 vueltas completadas
   Mejor tiempo: 11.234 s
   Promedio: 12.456 s
======================================================================
   📋 TIEMPOS POR VUELTA:
      Vuelta  1: 13.456 s
      Vuelta  2: 12.789 s
      Vuelta  3: 12.345 s
      Vuelta  4: 11.234 s  ⭐ Mejor
      Vuelta  5: 12.567 s
      Vuelta  6: 12.890 s
      Vuelta  7: 12.456 s
      Vuelta  8: 12.678 s
      Vuelta  9: 12.345 s
      Vuelta 10: 12.800 s
======================================================================
```

---

## 📊 Resultados

### Rendimiento Típico

Con los parámetros predeterminados, el controlador logra:

- ✅ **Tasa de éxito**: 100% (10 vueltas completas sin colisiones)
- ⏱️ **Tiempo promedio por vuelta**: ~30-31 segundos (depende de la pista)
- 🏎️ **Velocidad máxima alcanzada**: 9.0 m/s
- 🔄 **Frecuencia de control**: ~40 Hz (sincronizado con LiDAR)

### Comportamiento en Diferentes Escenarios

#### Rectas Largas
- Alcanza velocidad máxima (9.0 m/s)
- Modo: `STRAIGHT`
- Control estable con mínimas correcciones

#### Curvas Cerradas
- Reduce velocidad (2.8-3.5 m/s)
- Modo: `SHARP_TURN` o `TURN`
- Giros controlados sin derrapes

#### Obstáculos Cercanos
- Activación de modo `EMERGENCY`
- Maniobra evasiva inmediata
- Recuperación rápida al modo normal

---

## 🐛 Solución de Problemas

### El vehículo no se mueve

**Problema**: El nodo está corriendo pero el vehículo permanece quieto.

**Solución**:
```bash
# Verificar que el simulador está publicando
ros2 topic hz /scan

# Verificar que el nodo está publicando comandos
ros2 topic hz /drive

# Verificar parámetros de velocidad
ros2 param get /follow_gap_node max_speed
```

### El vehículo choca constantemente

**Problema**: El algoritmo no evita obstáculos correctamente.

**Solución**:
```bash
# Aumentar el radio de la burbuja
ros2 param set /follow_gap_node bubble_radius 1.0

# Reducir velocidad máxima
ros2 param set /follow_gap_node max_speed 5.0
```

### Las vueltas no se cuentan correctamente

**Problema**: El contador de vueltas no incrementa o cuenta múltiples veces.

**Solución**:
```bash
# Verificar que la odometría está llegando
ros2 topic echo /ego_racecar/odom

# Ajustar umbrales de detección
ros2 param set /follow_gap_node lap_detection_threshold 1.5
ros2 param set /follow_gap_node halfway_distance 3.0
```

### El vehículo oscila demasiado

**Problema**: Comportamiento zigzagueante en rectas.

**Solución**:
```bash
# Reducir la reactividad
ros2 param set /follow_gap_node smoothing_alpha 0.7

# Limitar la tasa de cambio de dirección
ros2 param set /follow_gap_node max_steering_rate 3.0
```

---

## 📝 Notas Adicionales

### Topics ROS2 Utilizados

| Topic | Tipo | Dirección | Descripción |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Entrada | Datos del LiDAR |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | Entrada | Odometría del vehículo |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | Salida | Comandos de control |

### Arquitectura del Sistema

```
┌─────────────────┐
│   Simulador     │
│   F1Tenth       │
└────────┬────────┘
         │
         ├── /scan (LaserScan)
         │
         └── /ego_racecar/odom (Odometry)
                  │
                  ↓
         ┌────────────────────┐
         │  FollowGapNode     │
         │  - Algoritmo FTG   │
         │  - Control velocidad│
         │  - Conteo vueltas  │
         └────────┬───────────┘
                  │
                  └── /drive (AckermannDriveStamped)
                           │
                           ↓
                  ┌────────────────┐
                  │   Simulador    │
                  │   (Control)    │
                  └────────────────┘
```

---

## 📚 Referencias

- [F1Tenth Official Documentation](https://f1tenth.org/)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)

---

## 👥 Autor

**Proyecto F1Tenth - Dereck Santander**

---