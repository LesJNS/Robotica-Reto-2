#!/usr/bin/env python3
"""
lane_controller.py — Nodo ROS2 controlador PID + feed-forward (RC-2).

Recibe el error lateral del carril y calcula la velocidad angular
del robot usando un controlador PID con término feed-forward predictivo.

Tópicos suscritos:
    /lane_error  (std_msgs/Float32) — error lateral en metros

Tópicos publicados:
    /cmd_vel     (geometry_msgs/Twist) — velocidad lineal y angular del robot

Parámetros (configurables desde pid_params.yaml):
    kp             — ganancia proporcional
    ki             — ganancia integral
    kd             — ganancia derivativa
    kff            — ganancia feed-forward de tendencia
    linear_speed   — velocidad lineal constante (m/s)
    max_angular    — límite de velocidad angular (rad/s)
    integral_limit — anti-windup: límite del término integral acumulado
    history_size   — muestras para calcular la tendencia del error
    turn_threshold — |ω| mínimo para aplicar feed-forward (evitarlo en recta)

Convención de signo:
    error > 0  →  robot está a la IZQUIERDA del centro  →  ω < 0 (girar derecha)
    error < 0  →  robot está a la DERECHA del centro    →  ω > 0 (girar izquierda)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')

        # ── Declarar parámetros con valores por defecto ────────────────────────
        self.declare_parameters('', [
            ('kp',             2.5),
            ('ki',             0.0),
            ('kd',             0.3),
            ('kff',            1.0),     # ganancia del término feed-forward
            ('linear_speed',   0.20),    # velocidad mínima requerida por RC-2 ≥ 0.20 m/s
            ('max_angular',    2.0),
            ('integral_limit', 0.5),     # anti-windup: máximo valor absoluto del integrador
            ('error_timeout',  0.5),     # segundos sin recibir error antes de parar
            ('control_rate',   30.0),    # frecuencia del loop de control en Hz
            ('recovery_w',     0.6),     # velocidad angular de recuperación (sin línea)
            ('recovery_v',     0.0),     # velocidad lineal de recuperación
            ('history_size',   10),      # muestras para calcular tendencia (~0.33 s a 30 Hz)
            ('turn_threshold',  0.3),    # |ω| mínimo para habilitar el FF en curvas
        ])

        gp = self.get_parameter
        self.kp          = float(gp('kp').value)
        self.ki          = float(gp('ki').value)
        self.kd          = float(gp('kd').value)
        self.kff         = float(gp('kff').value)
        self.v           = float(gp('linear_speed').value)
        self.max_w       = float(gp('max_angular').value)
        self.i_limit     = float(gp('integral_limit').value)
        self.timeout     = float(gp('error_timeout').value)
        self.recovery_w     = float(gp('recovery_w').value)
        self.recovery_v     = float(gp('recovery_v').value)
        self.turn_threshold = float(gp('turn_threshold').value)
        hist                = int(gp('history_size').value)
        rate                = float(gp('control_rate').value)

        # ── Estado interno del controlador ────────────────────────────────────
        self.error        = None    # último error recibido (metros)
        self.last_error   = 0.0    # error en el ciclo anterior (para el término D)
        self.last_w       = 0.0    # velocidad angular anterior (para decidir si aplicar FF)
        self.integral     = 0.0    # acumulador del término integral
        self.initialized  = False  # True tras recibir el primer error válido
        self.has_line     = False  # True si el último error era numérico (no NaN)
        self.last_stamp   = self.get_clock().now()
        self.last_rx      = self.get_clock().now()

        # Cola circular para calcular la tendencia del error (feed-forward)
        self.error_history = deque(maxlen=hist)

        # ── Tópicos ROS ───────────────────────────────────────────────────────
        self.sub   = self.create_subscription(
            Float32, '/lane_error', self.on_error, 10)
        self.pub   = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer que ejecuta el loop de control a la frecuencia configurada
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info('lane_controller listo.')
        self.get_logger().info(
            f'PID kp={self.kp} ki={self.ki} kd={self.kd} kff={self.kff}  '
            f'v={self.v} m/s  max_w={self.max_w} rad/s')

    def on_error(self, msg):
        """Recibe el error lateral desde lane_detector."""
        self.last_rx = self.get_clock().now()
        if not math.isnan(msg.data):
            # Error numérico: hay línea detectada
            self.error       = msg.data
            self.initialized = True
            self.has_line    = True
        else:
            # NaN: sin líneas o fin de carrera → parar
            self.has_line = False

    def _trend(self):
        """
        Calcula la pendiente media del historial de errores.

        Un error creciente (pendiente positiva) indica que el robot se aleja
        a la derecha → el feed-forward anticipa y gira a la derecha.
        Unidades: metros por muestra de control.
        """
        n = len(self.error_history)
        if n < 3:
            return 0.0
        lst = list(self.error_history)
        return (lst[-1] - lst[0]) / n

    def control_loop(self):
        """
        Loop principal del controlador PID + FF.

        Se ejecuta a control_rate Hz independientemente de los mensajes
        de /lane_error, lo que garantiza un control suave.
        """
        now = self.get_clock().now()
        dt  = (now - self.last_stamp).nanoseconds * 1e-9
        self.last_stamp = now

        if dt <= 0.0:
            return  # evitar división por cero en el primer tick

        # Esperar a tener al menos un error válido antes de arrancar
        if not self.initialized:
            self.pub.publish(Twist())
            return

        # Sin línea detectada → parar completamente y limpiar el estado PID
        if not self.has_line:
            self.integral = 0.0
            self.error_history.clear()
            self.pub.publish(Twist())
            return

        e = self.error
        self.error_history.append(e)

        # ── Términos PID ──────────────────────────────────────────────────────
        # P: respuesta proporcional al error actual
        P = self.kp * e

        # I: acumulación del error en el tiempo con anti-windup
        self.integral += e * dt
        self.integral  = max(-self.i_limit, min(self.i_limit, self.integral))
        I = self.ki * self.integral

        # D: respuesta a la velocidad de cambio del error
        derivative = (e - self.last_error) / dt
        D = self.kd * derivative

        # FF: solo en curvas (cuando el robot ya está girando significativamente)
        # En recta (ω ≈ 0) no se aplica para evitar oscilar en posición centrada
        trend = self._trend()
        FF = self.kff * trend if abs(self.last_w) > self.turn_threshold else 0.0

        # ω total: negado porque error > 0 → robot a la izquierda → girar derecha (ω < 0)
        w = -(P + I + D + FF)
        w = max(-self.max_w, min(self.max_w, w))  # saturar por seguridad

        cmd           = Twist()
        cmd.linear.x  = self.v    # velocidad lineal constante
        cmd.angular.z = w
        self.pub.publish(cmd)

        self.last_error = e
        self.last_w     = w


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Asegurar que el robot se detiene al salir
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
