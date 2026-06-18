#!/usr/bin/env python3
"""
cam_pub.py — Nodo ROS2 publicador de cámara.

Abre la cámara USB del robot (/dev/video0 por defecto) y publica
cada frame como sensor_msgs/Image en el tópico /image_raw.

Tópicos publicados:
    /image_raw  (sensor_msgs/Image, bgr8)

Parámetros (declarados en el nodo, configurables desde el launch):
    device  — índice de la cámara  (default: 0  → /dev/video0)
    width   — ancho del frame      (default: 640 px)
    height  — alto del frame       (default: 480 px)
    fps     — frecuencia de captura (default: 30 Hz)
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CamPub(Node):
    def __init__(self):
        super().__init__('cam_pub')

        # Declarar parámetros con valores por defecto
        self.declare_parameter('device', 0)
        self.declare_parameter('width',  640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps',    30)

        device = self.get_parameter('device').value
        width  = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps    = self.get_parameter('fps').value

        self.bridge = CvBridge()

        # Publisher en /image_raw — queue_size=10 para no saturar
        self.pub = self.create_publisher(Image, '/image_raw', 10)

        # Abrir la cámara por índice de dispositivo
        self.cap = cv2.VideoCapture(device)

        # MJPG: compresión por hardware en la cámara; reduce carga USB y CPU
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        if not self.cap.isOpened():
            self.get_logger().fatal(f'No se puede abrir /dev/video{device}')
            raise RuntimeError('camera not found')

        self.get_logger().info(
            f'cam_pub: /dev/video{device}  {width}x{height}@{fps}fps → /image_raw')

        # Timer que dispara _publish() a la frecuencia pedida
        self.timer = self.create_timer(1.0 / fps, self._publish)

    def _publish(self):
        """Captura un frame y lo publica como ROS Image."""
        ret, frame = self.cap.read()
        if not ret:
            # La cámara puede devolver False transitoriamente; se reintenta
            self.get_logger().warn('Frame vacío — reintentando')
            return

        # Convertir el frame OpenCV (numpy array) a mensaje ROS
        msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def destroy_node(self):
        """Liberar la cámara antes de apagar el nodo."""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CamPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
