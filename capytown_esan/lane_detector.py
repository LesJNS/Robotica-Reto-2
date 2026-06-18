#!/usr/bin/env python3
"""
lane_detector.py — Nodo ROS2 de detección de carril (RC-2).

Detecta las líneas amarilla (izquierda) y blanca (derecha) usando
filtrado HSV + IPM (Inverse Perspective Mapping) y calcula el error
lateral del robot respecto al centro del carril.

Pipeline:
    /image_raw  →  warp IPM  →  filtro HSV  →  centroides  →  /lane_error

Tópicos suscritos:
    /image_raw           (sensor_msgs/Image)

Tópicos publicados:
    /lane_error          (std_msgs/Float32)  — error lateral en metros
    /lane/debug_image    (sensor_msgs/Image) — imagen de diagnóstico
    /lane/yellow_x       (std_msgs/Float32)  — posición X amarillo (0-1)
    /lane/white_x        (std_msgs/Float32)  — posición X blanco   (0-1)

Convención de signo del error:
    error > 0  →  centro a la DERECHA del robot  →  girar derecha (ω < 0)
    error < 0  →  centro a la IZQUIERDA del robot →  girar izquierda (ω > 0)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.bridge = CvBridge()

        # ── Declarar todos los parámetros con valores por defecto ──────────────
        self.declare_parameters('', [
            # Blanco — baja saturación + alta luminosidad en HSV
            ('white_s_min',           0),
            ('white_s_max',           65),    # blanco real: S < 65
            ('white_v_min',           170),   # blanco brillante: V > 170
            ('white_v_max',           255),
            ('white_max_area',        25000),
            ('white_min_area',        1000),  # filtra reflejos puntuales pequeños
            ('white_min_elongation',  5.0),   # cintas son alargadas, reflejos no
            # Amarillo — rango hue cálido + saturación media-alta
            ('yellow_h_min',          15),
            ('yellow_h_max',          45),
            ('yellow_s_min',          45),    # ampliado para amarillo desaturado
            ('yellow_s_max',          255),
            ('yellow_v_min',          80),
            ('yellow_v_max',          255),
            ('yellow_min_area',       500),
            ('yellow_min_elongation', 8.0),   # cintas amarillas son muy alargadas
            # Geometría de la cámara / IPM
            ('min_area',           150),
            ('px_per_meter',       600.0),    # calibrado para la pista real
            ('look_ahead_row',     0.88),     # fracción de altura donde se mide (0=arriba, 1=abajo)
            ('band_half_height',   30),       # semialtura de la banda de medición en px
            # Setpoint de navegación
            ('yellow_setpoint',    0.33),     # fracción del ancho objetivo para el amarillo en curva
            # Comportamiento
            ('require_both_lines', False),    # False: navegar solo con amarillo en curvas
            ('publish_debug',      True),
            # Contador de vueltas: detener tras N curvas (4 curvas ≈ 1 vuelta)
            ('curves_to_stop',     12),       # 3 vueltas × 4 curvas = 12
            ('curve_debounce',      8),       # frames consecutivos sin blanco para confirmar curva
        ])

        gp = self.get_parameter

        # ── Rangos HSV como arrays numpy para cv2.inRange ─────────────────────
        # Blanco: cualquier hue (H=0-179), saturación baja, valor alto
        self.white_lo_hsv = np.array(
            [0,   gp('white_s_min').value, gp('white_v_min').value], dtype=np.uint8)
        self.white_hi_hsv = np.array(
            [179, gp('white_s_max').value, gp('white_v_max').value], dtype=np.uint8)

        self.white_max_area       = float(gp('white_max_area').value)
        self.white_min_area       = float(gp('white_min_area').value)
        self.white_min_elongation = float(gp('white_min_elongation').value)
        self.yellow_min_area      = float(gp('yellow_min_area').value)
        self.yellow_min_elongation= float(gp('yellow_min_elongation').value)

        # Amarillo: rango hue 15-45, saturación y valor medios-altos
        self.yellow_lo = np.array([gp('yellow_h_min').value,
                                    gp('yellow_s_min').value,
                                    gp('yellow_v_min').value], dtype=np.uint8)
        self.yellow_hi = np.array([gp('yellow_h_max').value,
                                    gp('yellow_s_max').value,
                                    gp('yellow_v_max').value], dtype=np.uint8)

        self.min_area        = float(gp('min_area').value)
        self.px_per_meter    = float(gp('px_per_meter').value)
        self.look_ahead_row  = float(gp('look_ahead_row').value)
        self.band_half_h     = int(gp('band_half_height').value)
        self.yellow_setpoint = float(gp('yellow_setpoint').value)
        self.require_both    = bool(gp('require_both_lines').value)
        self.publish_debug   = bool(gp('publish_debug').value)
        self.curves_to_stop  = int(gp('curves_to_stop').value)
        self.curve_debounce  = int(gp('curve_debounce').value)

        # ── Estado interno para contar curvas y detectar fin de carrera ────────
        self._white_missing_frames = 0   # frames consecutivos sin blanco detectado
        self._in_curve             = False
        self._curve_count          = 0
        self._finished             = False  # True tras completar las N vueltas

        # Matriz IPM — se inicializa en el primer frame (necesita w, h)
        self.M         = None
        self.warp_size = None

        # ── Tópicos ROS ────────────────────────────────────────────────────────
        self.sub     = self.create_subscription(
            Image, '/image_raw', self.on_image, 10)
        self.pub_err = self.create_publisher(Float32, '/lane_error',        10)
        self.pub_dbg = self.create_publisher(Image,   '/lane/debug_image',  10)
        self.pub_yx  = self.create_publisher(Float32, '/lane/yellow_x',     10)
        self.pub_wx  = self.create_publisher(Float32, '/lane/white_x',      10)

        self.get_logger().info('lane_detector listo.')
        self.get_logger().info(
            f'yellow HSV [{self.yellow_lo}] - [{self.yellow_hi}]  '
            f'white HSV [{self.white_lo_hsv}] - [{self.white_hi_hsv}]')

    # ── IPM (Inverse Perspective Mapping) ─────────────────────────────────────
    def build_ipm(self, w, h):
        """
        Calcula la homografía para transformar la vista de cámara en vista de pájaro.

        El trapecio src cubre la región de interés de la pista vista desde la cámara.
        El rectángulo dst es la proyección ortogonal de esa región.

        Puntos src (fracción de w, h):
            Superior-izq: (0.20, 0.55)  Superior-der: (0.80, 0.55)
            Inferior-der: (1.00, 0.97)  Inferior-izq: (0.00, 0.97)

        Puntos dst (vista pájaro centrada):
            Superior-izq: (0.25w, 0)   Superior-der: (0.75w, 0)
            Inferior-der: (0.75w, h)   Inferior-izq: (0.25w, h)
        """
        src = np.float32([
            [0.20 * w, 0.55 * h],  # esquina superior izquierda del trapecio
            [0.80 * w, 0.55 * h],  # esquina superior derecha
            [1.00 * w, 0.97 * h],  # esquina inferior derecha (borde de imagen)
            [0.00 * w, 0.97 * h],  # esquina inferior izquierda
        ])
        dst = np.float32([
            [0.25 * w, 0.0],       # vista pájaro: comprime lados laterales
            [0.75 * w, 0.0],
            [0.75 * w,  h],
            [0.25 * w,  h],
        ])
        self.M         = cv2.getPerspectiveTransform(src, dst)
        self.warp_size = (w, h)

    # ── Callback principal — procesamiento de cada frame ──────────────────────
    def on_image(self, msg):
        """Procesa un frame: IPM → detección HSV → error lateral → publicación."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        h, w = frame.shape[:2]

        # Calcular la homografía IPM solo una vez (tamaño de frame fijo)
        if self.M is None:
            self.build_ipm(w, h)

        # Transformar a vista de pájaro
        warp = cv2.warpPerspective(frame, self.M, self.warp_size)

        # ── Detección de colores en HSV ────────────────────────────────────────
        hsv            = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
        mask_yellow    = cv2.inRange(hsv, self.yellow_lo,    self.yellow_hi)
        mask_white_raw = cv2.inRange(hsv, self.white_lo_hsv, self.white_hi_hsv)

        # Morfología: eliminar ruido puntual y rellenar huecos pequeños en las cintas
        kernel = np.ones((3, 3), np.uint8)
        mask_yellow    = cv2.morphologyEx(mask_yellow,    cv2.MORPH_OPEN,  kernel)
        mask_yellow    = cv2.morphologyEx(mask_yellow,    cv2.MORPH_CLOSE, kernel)
        mask_white_raw = cv2.morphologyEx(mask_white_raw, cv2.MORPH_OPEN,  kernel)
        mask_white_raw = cv2.morphologyEx(mask_white_raw, cv2.MORPH_CLOSE, kernel)

        # Amarillo: sin restricción de zona — su hue es específico y no se confunde
        # Blanco: restringido a la mitad derecha — evita reflejos del suelo en la izquierda
        right_zone = np.zeros((h, w), dtype=np.uint8)
        right_zone[:, w // 2:] = 255
        mask_white_raw = cv2.bitwise_and(mask_white_raw, right_zone)

        # Eliminar píxeles amarillos que se cuelen en la máscara blanca
        mask_white_raw = cv2.bitwise_and(mask_white_raw,
                                          cv2.bitwise_not(mask_yellow))

        # ── Filtrar por forma: solo blobs alargados (cintas), no reflejos ─────
        mask_yellow = self._filter_by_shape(
            mask_yellow,
            min_area=self.yellow_min_area,
            max_area=self.white_max_area,
            min_elongation=self.yellow_min_elongation)

        mask_white = self._filter_by_shape(
            mask_white_raw,
            min_area=self.white_min_area,
            max_area=self.white_max_area,
            min_elongation=self.white_min_elongation,
            min_cx_ratio=0.45)  # el blanco debe estar en el lado derecho

        # ── Centroides en la banda de look-ahead ──────────────────────────────
        # Solo se mide en una banda horizontal a look_ahead_row de altura
        row  = int(self.look_ahead_row * h)
        band = slice(max(0, row - self.band_half_h),
                     min(h, row + self.band_half_h))

        x_yellow = self._centroid_x(mask_yellow[band, :])
        x_white  = self._centroid_x(mask_white[band, :])

        # ── Cálculo del error lateral ──────────────────────────────────────────
        # Modo RECTA  (amarillo + blanco visibles): centro exacto entre ambas líneas
        # Modo CURVA  (solo amarillo):              amarillo como referencia fija
        # Modo NINGUNA:                             NaN → el controller para el robot
        error_px = None

        if x_yellow is not None and x_white is not None:
            # Recta: el carril lo definen las dos líneas
            error_px = (x_yellow + x_white) / 2.0 - w / 2.0

        elif x_yellow is not None:
            # Curva: mantener el amarillo en su posición objetivo (setpoint)
            error_px = x_yellow - self.yellow_setpoint * w

        elif x_white is not None and not self.require_both:
            # Solo blanco (situación poco frecuente): usar blanco como referencia
            error_px = x_white - (1.0 - self.yellow_setpoint) * w

        # Convertir de píxeles a metros usando la calibración px_per_meter
        error_m = error_px / self.px_per_meter if error_px is not None else float('nan')

        # ── Contador de curvas (para saber cuántas vueltas se han dado) ────────
        # Una curva se confirma cuando el blanco desaparece durante ≥ curve_debounce frames.
        # Al reaparecer el blanco se cierra la curva y se incrementa el contador.
        if not self._finished:
            if x_white is None:
                self._white_missing_frames += 1
                # Umbral superado → estamos entrando a una curva
                if self._white_missing_frames >= self.curve_debounce and not self._in_curve:
                    self._in_curve = True
            else:
                if self._in_curve:
                    # El blanco volvió → la curva terminó
                    self._curve_count += 1
                    self.get_logger().info(
                        f'Curva {self._curve_count}/{self.curves_to_stop} completada')
                    if self._curve_count >= self.curves_to_stop:
                        self._finished = True
                        self.get_logger().info('*** 3 VUELTAS COMPLETAS — deteniendo robot ***')
                self._in_curve = False
                self._white_missing_frames = 0  # reiniciar contador al ver blanco

        # Cuando se terminan las vueltas, emitir NaN permanente para que el controller pare
        if self._finished:
            error_m = float('nan')

        # ── Publicar resultados ────────────────────────────────────────────────
        out      = Float32()
        out.data = float(error_m)
        self.pub_err.publish(out)

        # Posición normalizada (0-1) de cada línea
        yx_msg      = Float32()
        yx_msg.data = float(x_yellow / w) if x_yellow is not None else float('nan')
        wx_msg      = Float32()
        wx_msg.data = float(x_white  / w) if x_white  is not None else float('nan')
        self.pub_yx.publish(yx_msg)
        self.pub_wx.publish(wx_msg)

        # Posición del centro estimado en píxeles (para la imagen de debug)
        center_px = (w / 2.0 + error_px) if error_px is not None else None

        if self.publish_debug:
            self._publish_debug(warp, mask_white, mask_yellow, row,
                                x_white, x_yellow, center_px, msg)

    # ── Filtrado por forma (PCA) ───────────────────────────────────────────────
    def _filter_by_shape(self, mask, min_area, max_area, min_elongation,
                         min_cx_ratio=0.0, h_total=0, min_cy=0):
        """
        Conserva blobs con forma alargada (como cintas de líneas de carril).
        Rechaza reflejos puntuales (circulares) y ruido pequeño.

        Usa PCA (análisis de componentes principales) para calcular la elongación:
            elongation = eigenvalue_1 / eigenvalue_2
        Un círculo da elongation ≈ 1; una cinta larga da elongation >> 1.

        Args:
            mask:           máscara binaria de entrada
            min_area:       área mínima del blob (pixels²)
            max_area:       área máxima del blob (pixels²)
            min_elongation: ratio PCA mínimo para considerar el blob como cinta
            min_cx_ratio:   fracción mínima del ancho donde debe estar el centroide X
        """
        result = np.zeros_like(mask)
        h, w = mask.shape[:2]

        # Etiquetar componentes conexas y obtener sus estadísticas
        num, labels, stats, cents = cv2.connectedComponentsWithStats(
            mask, connectivity=8)

        for i in range(1, num):  # i=0 es el fondo
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area or area > max_area:
                continue

            cx, cy = cents[i]

            # Descartar blobs en la zona izquierda (solo para blanco)
            if cx < min_cx_ratio * w:
                continue
            if cy < min_cy:
                continue

            # Calcular elongación con PCA si hay suficientes puntos
            pts = np.column_stack(np.where(labels == i))
            if len(pts) > 10:
                xy = pts[:, ::-1].astype(np.float32)   # (col, row) = (x, y)
                _, _, eigval = cv2.PCACompute2(xy, mean=None)
                elongation = float(eigval[0, 0] / (eigval[1, 0] + 1e-6))
                if elongation < min_elongation:
                    continue  # blob demasiado redondo → es un reflejo

            result[labels == i] = 255

        return result

    @staticmethod
    def _centroid_x(mask):
        """Devuelve la coordenada X del centroide de la máscara, o None si está vacía."""
        m = cv2.moments(mask, binaryImage=True)
        if m['m00'] < 1e-3:
            return None
        return m['m10'] / m['m00']

    # ── Imagen de diagnóstico ─────────────────────────────────────────────────
    def _publish_debug(self, warp, mask_white, mask_yellow, row,
                       xw, xy, xc, header_msg):
        """
        Genera y publica la imagen de debug (/lane/debug_image) con:
            - Overlay de máscaras detectadas (cyan=amarillo, azul claro=blanco)
            - Línea de look-ahead (verde)
            - Centroides de cada línea y del centro calculado
            - Texto con el estado de detección
        """
        h, w = warp.shape[:2]
        dbg = warp.copy()

        # Overlay semitransparente: mezcla 50% imagen original + 50% colores de máscara
        overlay = dbg.copy()
        overlay[mask_yellow > 0] = (0, 220, 220)    # cyan = amarillo detectado
        overlay[mask_white  > 0] = (200, 200, 255)  # azul claro = blanco detectado
        cv2.addWeighted(overlay, 0.5, dbg, 0.5, 0, dbg)

        # Línea horizontal de look-ahead y línea vertical central
        cv2.line(dbg, (0, row), (w, row), (0, 255, 0), 1)
        cv2.line(dbg, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)

        # Puntos de centroide: W=blanco, Y=amarillo, C=centro calculado
        for x, color, label in (
            (xw, (255, 255, 255), 'W'),   # blanco → punto blanco
            (xy, (0, 200, 255),   'Y'),   # amarillo → punto cyan
            (xc, (0, 0, 255),     'C'),   # centro → punto rojo
        ):
            if x is not None:
                cv2.circle(dbg, (int(x), row), 7, color, -1)
                cv2.putText(dbg, label, (int(x) + 9, row - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Estado de detección en la esquina superior izquierda
        detected = []
        if xy is not None:
            detected.append('Y')
        if xw is not None:
            detected.append('W')
        status = '+'.join(detected) if detected else 'NONE'
        cv2.putText(dbg, f'Lines: {status}', (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        out        = self.bridge.cv2_to_imgmsg(dbg, 'bgr8')
        out.header = header_msg.header
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
