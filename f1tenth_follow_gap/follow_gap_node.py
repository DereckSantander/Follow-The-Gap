#!/usr/bin/env python3
"""
Follow the Gap Algorithm - F1Tenth Autonomous Racing Controller

Este nodo implementa el algoritmo "Follow the Gap" para navegación reactiva
en carreras autónomas F1Tenth, incluyendo sistema de conteo de vueltas y cronometraje.

Author: Proyecto F1Tenth
License: MIT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import numpy as np
import time


class FollowGapNode(Node):
    """
    Nodo ROS2 que implementa el algoritmo Follow the Gap para navegación reactiva.
    
    El algoritmo funciona en tres pasos principales:
    1. Crear una "burbuja" alrededor del obstáculo más cercano
    2. Encontrar el gap (espacio libre) más grande
    3. Dirigirse hacia el centro de ese gap
    """
    
    def __init__(self):
        super().__init__('follow_gap_node')
        
        # ==================== VARIABLES DE ESTADO ====================
        self.last_speed = 0.0
        self.prev_angle = 0.0
        self.last_lidar_time = time.time()
        self.mode = "INIT"
        self.position = (0.0, 0.0)
        
        # ==================== SISTEMA DE VUELTAS ====================
        self.lap_count = 0
        self.lap_times = []
        self.start_time = time.time()
        self.last_lap_time = self.start_time
        self.start_position = None  # Línea de meta (se establece con primera odometría)
        self.lap_detection_threshold = 1.0  # Distancia para detectar cruce de meta (m)
        self.crossed_halfway = False  # Asegura que completó toda la vuelta
        self.halfway_distance = 2.5  # Distancia para considerar que salió de zona inicio (m)
        self.in_finish_zone = False  # Evita contar múltiples veces
        self.race_finished = False  # Indica que completó las 10 vueltas
        
        # ==================== PARÁMETROS DEL ALGORITMO ====================
        self.declare_parameter('max_speed', 7.0)
        self.declare_parameter('min_speed', 2.8)
        self.declare_parameter('bubble_radius', 0.7)
        self.declare_parameter('gap_threshold', 1.5)
        self.declare_parameter('smoothing_alpha', 0.85)
        self.declare_parameter('angle_deadband', 0.0)
        self.declare_parameter('max_steering_rate', 5.0)
        self.declare_parameter('disparity_threshold', 0.5)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.gap_threshold = self.get_parameter('gap_threshold').value
        self.alpha = self.get_parameter('smoothing_alpha').value
        self.angle_deadband = self.get_parameter('angle_deadband').value
        self.max_steering_rate = self.get_parameter('max_steering_rate').value
        self.disparity_threshold = self.get_parameter('disparity_threshold').value
        
        # ==================== PUBLISHERS Y SUBSCRIBERS ====================
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.get_logger().info('🏁 Follow the Gap node initialized')
        self.get_logger().info('📡 Subscribed to /ego_racecar/odom and /scan')

    
    # ==================== CALLBACKS ====================
    
    def odom_callback(self, msg):
        """
        Callback de odometría para actualizar posición del robot.
        Establece la posición inicial como línea de meta en la primera llamada.
        """
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        self.position = (new_x, new_y)
        
        # Establecer posición inicial como línea de meta (solo la primera vez)
        if self.start_position is None:
            self.start_position = self.position
            self.get_logger().info(
                f"\033[1;32m📍 Línea de meta establecida en: x={self.start_position[0]:.2f}, "
                f"y={self.start_position[1]:.2f}\033[0m"
            )
            self.get_logger().info("\033[1;33m🔄 Sistema de vueltas inicializado\033[0m")
    
    # ==================== PROCESAMIENTO DE LIDAR ====================
    
    def preprocess_lidar(self, ranges, range_max):
        """
        Preprocesa los datos del LiDAR: limpia valores infinitos y limita rangos.
        
        Args:
            ranges: Array de rangos del LiDAR
            range_max: Rango máximo del sensor
            
        Returns:
            Array procesado de rangos
        """
        proc = np.array(ranges)
        proc = np.where(np.isfinite(proc), proc, range_max)
        proc = np.clip(proc, 0, range_max)
        return proc

    
    # ==================== ALGORITMO FOLLOW THE GAP ====================

    def find_max_gap(self, free_space_ranges):
        """
        Encuentra el gap (espacio libre) más grande en los datos del LiDAR.
        
        Args:
            free_space_ranges: Array de rangos procesados del LiDAR
            
        Returns:
            Tupla (start_idx, end_idx) con los índices del gap más grande
        """
        mask = free_space_ranges > self.gap_threshold
        max_len = 0
        max_start = 0
        max_end = 0
        current_start = None
        
        for i, val in enumerate(mask):
            if val:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    length = i - current_start
                    if length > max_len:
                        max_len = length
                        max_start = current_start
                        max_end = i - 1
                    current_start = None
        
        # Verificar al final del array
        if current_start is not None:
            length = len(mask) - current_start
            if length > max_len:
                max_len = length
                max_start = current_start
                max_end = len(mask) - 1
        
        return max_start, max_end

    def find_best_point(self, start_idx, end_idx, ranges):
        """
        Encuentra el mejor punto objetivo dentro del gap.
        Utiliza el centro del gap como estrategia (Follow the Gap clásico).
        
        Args:
            start_idx: Índice de inicio del gap
            end_idx: Índice de fin del gap
            ranges: Array de rangos del LiDAR
            
        Returns:
            Índice del punto objetivo
        """
        if end_idx < start_idx:
            return len(ranges) // 2
        
        # Apuntar al centro del gap
        center = (start_idx + end_idx) // 2
        return center

    
    # ==================== CALLBACK PRINCIPAL DE LIDAR ====================

    def lidar_callback(self, msg):
        """
        Callback principal que ejecuta el algoritmo Follow the Gap y controla el vehículo.
        
        Pasos del algoritmo:
        1. Preprocesar datos del LiDAR
        2. Crear burbuja alrededor del obstáculo más cercano
        3. Encontrar el gap más grande
        4. Calcular ángulo hacia el centro del gap
        5. Ajustar velocidad según situación
        6. Aplicar suavizado y publicar comando de control
        7. Detectar cruce de línea de meta para conteo de vueltas
        """
        # Validación de atributos (seguridad ante reinicialización)
        if not hasattr(self, 'last_lidar_time'):
            self.last_lidar_time = time.time()
        if not hasattr(self, 'prev_angle'):
            self.prev_angle = 0.0
        if not hasattr(self, 'alpha'):
            self.alpha = 0.6
        
        try:
            # Paso 1: Preprocesar datos del LiDAR
            ranges = self.preprocess_lidar(msg.ranges, msg.range_max)
            angle_increment = msg.angle_increment
            now_time = time.time()
            dt = max(1e-3, now_time - self.last_lidar_time)
            self.last_lidar_time = now_time
            
            # Paso 2: Crear "burbuja" alrededor del obstáculo más cercano
            min_idx = np.argmin(ranges)
            bubble_size = int(self.bubble_radius / angle_increment)
            bubble_start = max(0, min_idx - bubble_size)
            bubble_end = min(len(ranges) - 1, min_idx + bubble_size)
            ranges[bubble_start:bubble_end+1] = 0
            
            # Paso 3: Determinar estrategia (emergencia o Follow the Gap)
            min_dist = np.min(ranges[ranges > 0])
            emergency = False
            
            if min_dist < 0.3:
                # Evasión de emergencia si está muy cerca de un obstáculo
                emergency = True
                min_idx = np.argmin(ranges)
                if min_idx < len(ranges) // 2:
                    angle = 0.8
                else:
                    angle = -0.8
                speed = self.min_speed * 0.3
                self.mode = "EMERGENCY"
            else:
                # Paso 4: Encontrar el gap más grande y calcular ángulo
                gap_start, gap_end = self.find_max_gap(ranges)
                if gap_end < gap_start:
                    target_idx = np.argmax(ranges)
                else:
                    target_idx = self.find_best_point(gap_start, gap_end, ranges)
                
                angle = msg.angle_min + target_idx * angle_increment
                distance = ranges[target_idx] if target_idx < len(ranges) else 1.0
                steering_abs = abs(angle)
                
                # Paso 5: Control de velocidad adaptativo
                if distance < 0.8:
                    speed = self.min_speed * 0.8
                    self.mode = "SLOW"
                elif steering_abs > 0.4:
                    speed = self.min_speed * 1.2
                    self.mode = "SHARP_TURN"
                elif steering_abs > 0.25:
                    speed = self.min_speed + (self.max_speed - self.min_speed) * 0.5
                    self.mode = "TURN"
                elif steering_abs > 0.12:
                    speed = self.min_speed + (self.max_speed - self.min_speed) * 0.75
                    self.mode = "CURVE"
                else:
                    speed = self.max_speed
                    self.mode = "STRAIGHT"
            
            raw_angle = angle
            
            # Paso 6: Suavizado del ángulo de dirección
            smoothed_angle = self.alpha * raw_angle + (1 - self.alpha) * self.prev_angle
            
            # Limitación de tasa de cambio
            max_delta = self.max_steering_rate * dt
            delta = smoothed_angle - self.prev_angle
            if abs(delta) > max_delta:
                smoothed_angle = self.prev_angle + np.sign(delta) * max_delta
            
            angle = smoothed_angle
            self.prev_angle = angle
            self.last_speed = speed
            
            # Publicar comando de control
            drive_msg = AckermannDriveStamped()
            
            if self.lap_count >= 10:
                # Detener el vehículo al completar 10 vueltas
                drive_msg.drive.steering_angle = 0.0
                drive_msg.drive.speed = 0.0
                self.mode = "FINISHED"
            else:
                drive_msg.drive.steering_angle = float(angle)
                drive_msg.drive.speed = float(speed)
            
            self.drive_pub.publish(drive_msg)
            
            # Paso 7: Sistema de detección de vueltas
            self._detect_lap_completion()
            
            # Mostrar logs de estado si la carrera no ha finalizado
            if not self.race_finished:
                self._display_vehicle_status(angle, raw_angle, emergency)
                
        except Exception as e:
            self.get_logger().error(f"Error en lidar_callback: {e}")
            # Estado seguro en caso de error
            safe = AckermannDriveStamped()
            safe.drive.steering_angle = 0.0
            safe.drive.speed = 0.0
            self.drive_pub.publish(safe)
    
    # ==================== SISTEMA DE DETECCIÓN DE VUELTAS ====================
    
    def _detect_lap_completion(self):
        """
        Detecta el cruce de la línea de meta para conteo de vueltas.
        
        Utiliza un sistema de estados:
        1. Robot sale de la zona de inicio (halfway_distance)
        2. Robot regresa a la zona de meta (lap_detection_threshold)
        3. Se cuenta la vuelta y se registra el tiempo
        """
        if self.start_position is None:
            return
        
        # Calcular distancia a la línea de meta
        distance_to_start = np.sqrt(
            (self.position[0] - self.start_position[0])**2 + 
            (self.position[1] - self.start_position[1])**2
        )
        
        # Marcar que salió de la zona de inicio (dio media vuelta o más)
        if distance_to_start > self.halfway_distance and not self.crossed_halfway:
            self.crossed_halfway = True
            self.get_logger().info(
                f"\033[1;36m➡️  Robot salió de zona de inicio (dist: {distance_to_start:.2f}m)\033[0m"
            )
        
        # Detectar entrada a zona de meta
        in_zone = distance_to_start < self.lap_detection_threshold
        
        # Contar vuelta: entró a zona de meta Y ya había salido Y no había sido contada
        if in_zone and self.crossed_halfway and not self.in_finish_zone:
            lap_now = time.time()
            lap_time = lap_now - self.last_lap_time
            
            # Registrar vuelta
            self.lap_count += 1
            self.lap_times.append(lap_time)
            self.last_lap_time = lap_now
            self.crossed_halfway = False
            self.in_finish_zone = True
            
            # Mostrar mensaje de vuelta completada
            self._display_lap_message(lap_time)
        
        # Salir de zona de meta
        if not in_zone and self.in_finish_zone:
            self.in_finish_zone = False
            self.get_logger().info(
                f"\033[1;35m➡️  Robot salió de zona de meta, iniciando vuelta #{self.lap_count}\033[0m"
            )
    
    # ==================== VISUALIZACIÓN Y LOGS ====================
    
    def _display_lap_message(self, lap_time):
        """
        Muestra mensaje formateado cuando se completa una vuelta.
        
        Args:
            lap_time: Tiempo de la vuelta completada en segundos
        """
        sep_line = "=" * 70
        color_green = "\033[1;32m"
        color_cyan = "\033[1;36m"
        color_yellow = "\033[1;33m"
        color_purple = "\033[1;35m"
        color_red = "\033[1;31m"
        color_white = "\033[1;97m"
        color_reset = "\033[0m"
        
        mejor_tiempo = min(self.lap_times) if self.lap_times else lap_time
        
        if self.lap_count >= 10:
            # Mensaje de fin de carrera con todos los tiempos
            self.race_finished = True
            tiempos_detallados = "\n"
            for i, t in enumerate(self.lap_times, 1):
                tiempos_detallados += f"{color_cyan}      Vuelta {i:2d}: {color_yellow}{t:.3f} s{color_reset}\n"
            
            self.get_logger().info(
                f"\n{sep_line}\n"
                f"{color_red}🏁🏁🏁 ¡CARRERA FINALIZADA! 🏁🏁🏁{color_reset}\n"
                f"{color_cyan}   10 vueltas completadas{color_reset}\n"
                f"{color_purple}   Mejor tiempo: {mejor_tiempo:.3f} s{color_reset}\n"
                f"{color_green}   Promedio: {sum(self.lap_times)/len(self.lap_times):.3f} s{color_reset}\n"
                f"{sep_line}\n"
                f"{color_white}   📋 TIEMPOS POR VUELTA:{color_reset}\n"
                f"{tiempos_detallados}"
                f"{sep_line}"
            )
        else:
            self.get_logger().info(
                f"\n{sep_line}\n"
                f"{color_green}🏁 ¡VUELTA COMPLETADA! 🏁{color_reset}\n"
                f"{color_cyan}   Vuelta #{self.lap_count} de 10{color_reset}\n"
                f"{color_yellow}   Tiempo de vuelta: {lap_time:.3f} segundos{color_reset}\n"
                f"{color_purple}   Mejor tiempo: {mejor_tiempo:.3f} s{color_reset}\n"
                f"{sep_line}"
            )
    
    def _display_vehicle_status(self, angle, raw_angle, emergency):
        """
        Muestra el estado actual del vehículo en la consola.
        
        Args:
            angle: Ángulo de dirección suavizado
            raw_angle: Ángulo de dirección sin suavizar
            emergency: Booleano indicando si está en modo emergencia
        """
        last_lap_str = "N/A"
        if len(self.lap_times) > 0:
            last_lap_str = f"{self.lap_times[-1]:.2f}s"
        
        # Códigos de color
        color_blue = "\033[1;34m"
        color_white = "\033[1;97m"
        color_cyan = "\033[0;36m"
        color_green = "\033[1;32m"
        color_yellow = "\033[1;33m"
        color_white2 = "\033[1;37m"
        color_purple = "\033[1;35m"
        color_red = "\033[1;31m"
        color_cyan2 = "\033[1;36m"
        color_gray = "\033[0;90m"
        color_reset = "\033[0m"
        
        emergency_text = f" {color_red}⚠️  EVASIÓN ⚠️{color_reset}" if emergency else ""
        
        status_text = ""
        if self.lap_count >= 10:
            status_text = f" {color_red}[CARRERA FINALIZADA]{color_reset}"
        
        self.get_logger().info(
            f"{color_blue}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{color_reset}\n"
            f"{color_white}📊 ESTADO DEL VEHÍCULO{color_reset}{status_text}\n"
            f"{color_cyan}  🔄 Vueltas:             {color_green}{self.lap_count}/10{color_reset}\n"
            f"{color_cyan}  ⏱️  Última vuelta:        {color_yellow}{last_lap_str}{color_reset}\n"
            f"{color_cyan}  📍 Posición:            {color_white2}x={self.position[0]:+7.2f}m, y={self.position[1]:+7.2f}m{color_reset}\n"
            f"{color_cyan}  🚗 Velocidad:           {color_green}{self.last_speed:.2f} m/s{color_reset}\n"
            f"{color_cyan}  🎯 Modo:                {color_purple}{self.mode:12s}{color_reset}{emergency_text}\n"
            f"{color_cyan}  🎮 Ángulo:              {color_cyan2}{angle:+.3f} rad{color_reset} {color_gray}(raw: {raw_angle:+.3f}){color_reset}\n"
            f"{color_blue}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{color_reset}"
        )


# ==================== MAIN ====================

def main(args=None):
    """Función principal para inicializar y ejecutar el nodo."""
    rclpy.init(args=args)
    node = FollowGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
