#!/usr/bin/env python3
"""
camera_stream.py — Nodo ROS2 + servidor web MJPEG en vivo.

Sirve una página HTML con 3 paneles de cámara en tiempo real en http://0.0.0.0:5800/

Paneles:
    1. /lane/raw_trapezoid  — imagen original con trapecio IPM dibujado
    2. /lane/birdeye_image  — vista de pájaro (IPM aplicado)
    3. /lane/debug_image    — máscaras de color + centroides

Fallback: si lane_detector no está corriendo, el panel 1 muestra /image_raw.

Acceso desde la red local:
    http://10.42.0.1:5800/          (desde tu PC, IP del robot)
    http://localhost:5800/           (desde dentro del Docker)

Streams MJPEG disponibles:
    /stream/raw  — panel 1
    /stream/bev  — panel 2
    /stream/dbg  — panel 3
"""

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from http.server import HTTPServer, BaseHTTPRequestHandler

PORT = 5800  # puerto HTTP — debe estar expuesto en el Docker

# Diccionario global con el último frame de cada panel (None = aún no recibido)
_frames = {'raw': None, 'bev': None, 'dbg': None}
_lock   = threading.Lock()   # protege _frames de acceso concurrente

# ── HTML de la página principal ───────────────────────────────────────────────
_HTML = """\
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>CapyTown RC-2 — camara en vivo</title>
<style>
  body  { background:#111; color:#eee; font-family:sans-serif;
          text-align:center; margin:0; padding:10px; }
  h1    { color:#F9A825; margin:8px 0; font-size:1.4em; }
  .panels { display:flex; justify-content:center; gap:8px; flex-wrap:wrap; }
  .panel  { display:flex; flex-direction:column; align-items:center; }
  img   { border:2px solid #333; max-width:100%; }
  .lbl  { font-size:0.8em; color:#aaa; margin-top:4px; }
  .hint { font-size:0.7em; color:#666; margin-top:8px; }
</style>
</head>
<body>
<h1>CapyTown RC-2 &mdash; camara en vivo</h1>
<div class="panels">
  <div class="panel">
    <img src="/stream/raw" width="426">
    <div class="lbl">1. Camara cruda + trapecio IPM</div>
  </div>
  <div class="panel">
    <img src="/stream/bev" width="426">
    <div class="lbl">2. Vista pajaro (IPM)</div>
  </div>
  <div class="panel">
    <img src="/stream/dbg" width="426">
    <div class="lbl">3. Mascara amarillo + blanco</div>
  </div>
</div>
<div class="hint">
  Panel 1: el cuadrilatero cyan debe cubrir ambas lineas de la pista
  &nbsp;|&nbsp;
  Panel 3: Blanco=borde &nbsp; Cyan=eje amarillo &nbsp; Rojo=centro &nbsp; Verde=look-ahead
</div>
</body>
</html>"""


class _Handler(BaseHTTPRequestHandler):
    """Manejador HTTP: sirve la página HTML y los streams MJPEG."""

    def log_message(self, *_):
        pass  # silenciar logs de acceso HTTP en consola

    def do_GET(self):
        if self.path == '/':
            # Página principal con los 3 paneles
            body = _HTML.encode()
            self.send_response(200)
            self.send_header('Content-Type',   'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path.startswith('/stream/'):
            # Stream MJPEG: bucle infinito enviando frames como multipart
            key = self.path.rsplit('/', 1)[-1]  # 'raw', 'bev' o 'dbg'
            if key not in _frames:
                self.send_response(404)
                self.end_headers()
                return

            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            try:
                while True:
                    with _lock:
                        f = _frames[key]
                    if f is not None:
                        # Comprimir a JPEG con calidad 75 (buen balance tamaño/calidad)
                        ok, jpg = cv2.imencode(
                            '.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 75])
                        if ok:
                            data = jpg.tobytes()
                            self.wfile.write(
                                b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n'
                                + data + b'\r\n')
                    time.sleep(0.04)  # ~25 fps máximo para no saturar la red
            except Exception:
                pass  # cliente desconectado — salir del bucle silenciosamente

        else:
            self.send_response(404)
            self.end_headers()


class CameraStream(Node):
    """Nodo ROS2 que suscribe a los tópicos de imagen y actualiza _frames."""

    def __init__(self):
        super().__init__('camera_stream')
        self.bridge   = CvBridge()
        self._has_raw = False  # True cuando ya llegó /lane/raw_trapezoid

        # Suscripción a los 3 tópicos de imagen del detector
        self.create_subscription(
            Image, '/lane/raw_trapezoid', self._make_cb('raw'), 5)
        self.create_subscription(
            Image, '/lane/birdeye_image', self._make_cb('bev'), 5)
        self.create_subscription(
            Image, '/lane/debug_image',   self._make_cb('dbg'), 5)

        # Fallback: si lane_detector no publica raw_trapezoid, usar la cámara cruda
        self.create_subscription(
            Image, '/image_raw', self._cb_fallback, 5)

        self.get_logger().info(
            f'camera_stream listo — abre http://10.42.0.1:{PORT}/ en el navegador')

    def _make_cb(self, key):
        """Genera un callback que guarda el frame en _frames[key]."""
        def _cb(msg):
            with _lock:
                _frames[key] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if key == 'raw':
                self._has_raw = True   # desactivar el fallback de /image_raw
        return _cb

    def _cb_fallback(self, msg):
        """Muestra /image_raw en el panel 1 solo si raw_trapezoid no está disponible."""
        if not self._has_raw:
            with _lock:
                _frames['raw'] = self.bridge.imgmsg_to_cv2(msg, 'bgr8')


def main(args=None):
    rclpy.init(args=args)
    node = CameraStream()

    # Lanzar el servidor HTTP en un hilo daemon (muere al terminar el proceso)
    server = HTTPServer(('0.0.0.0', PORT), _Handler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    print(f'[camera_stream] Servidor web en http://10.42.0.1:{PORT}/')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
