#!/usr/bin/env python3
"""
Genera lane_visualization_s11.png con análisis visual completo de la corrida.

Paneles:
  [1] Error lateral vs tiempo  (corrida completa)
  [2] Posición X del amarillo vs tiempo  (fracción del ancho 0-1)
  [3] Posición X del blanco   vs tiempo  (fracción del ancho 0-1)
  [4-7] 4 posiciones de la corrida: cámara normal vs cámara con detección (lado a lado)

Uso:
    python3 scripts/lane_visualization.py <ruta_al_bag>
    python3 scripts/lane_visualization.py ~/bags/lane_error_s11

El bag debe contener:
    /lane_error        (std_msgs/Float32)
    /lane/yellow_x     (std_msgs/Float32)   -- publicado por lane_detector v1.5+
    /lane/white_x      (std_msgs/Float32)
    /camera/image_raw  (sensor_msgs/Image)  -- cámara original
    /lane/debug_image  (sensor_msgs/Image)  -- cámara con detección
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import cv2

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float32
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    print("ERROR: ejecuta dentro del entorno ROS2:")
    print("       source /opt/ros/humble/setup.bash")
    sys.exit(1)

if len(sys.argv) < 2:
    print(f"Uso: python3 {sys.argv[0]} <ruta_bag> [salida.png]")
    sys.exit(1)

bag_path = sys.argv[1]
out_file  = sys.argv[2] if len(sys.argv) > 2 else "lane_visualization_s11.png"

if not os.path.exists(bag_path):
    print(f"ERROR: no existe el bag en '{bag_path}'")
    sys.exit(1)

# ── Leer bag ─────────────────────────────────────────────────────────────────
bridge  = CvBridge()
reader  = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions('', ''),
)

t0 = None
data = {
    'err':   {'t': [], 'v': []},
    'yx':    {'t': [], 'v': []},
    'wx':    {'t': [], 'v': []},
    'raw':   {'t': [], 'v': []},   # /camera/image_raw
    'dbg':   {'t': [], 'v': []},   # /lane/debug_image
}

TOPIC_MAP = {
    '/lane_error':       ('err',  Float32),
    '/lane/yellow_x':    ('yx',   Float32),
    '/lane/white_x':     ('wx',   Float32),
    '/camera/image_raw': ('raw',  Image),
    '/lane/debug_image': ('dbg',  Image),
}

while reader.has_next():
    topic, raw_data, stamp = reader.read_next()
    if topic not in TOPIC_MAP:
        continue
    if t0 is None:
        t0 = stamp
    t = (stamp - t0) * 1e-9

    key, msg_type = TOPIC_MAP[topic]
    msg = deserialize_message(raw_data, msg_type)

    if msg_type is Float32:
        data[key]['t'].append(t)
        data[key]['v'].append(msg.data)
    else:
        try:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            data[key]['t'].append(t)
            data[key]['v'].append(frame)
        except Exception:
            pass

for k in data:
    data[k]['t'] = np.array(data[k]['t']) if data[k]['t'] else np.array([])
    if k not in ('raw', 'dbg') and len(data[k]['v']) > 0:
        data[k]['v'] = np.array(data[k]['v'])

if len(data['err']['t']) == 0:
    print("ERROR: No se encontró /lane_error en el bag.")
    sys.exit(1)

t_dur  = max(
    data['err']['t'][-1] if len(data['err']['t']) else 0,
    data['yx']['t'][-1]  if len(data['yx']['t'])  else 0,
)
print(f"Duración: {t_dur:.1f}s")
print(f"  /lane_error    : {len(data['err']['t'])} muestras")
print(f"  /lane/yellow_x : {len(data['yx']['t'])} muestras")
print(f"  /lane/white_x  : {len(data['wx']['t'])} muestras")
print(f"  /camera/image_raw : {len(data['raw']['t'])} frames")
print(f"  /lane/debug_image: {len(data['dbg']['t'])} frames")

# ── Seleccionar 4 snapshots equiespaciados ────────────────────────────────────
FRACS    = [0.15, 0.38, 0.62, 0.85]
LABELS   = ['Inicio (15%)', 'Curva 1 (38%)', 'Recta 2 (62%)', 'Curva 2 (85%)']
snaps    = []   # lista de (t, frame_raw, frame_dbg)

for frac in FRACS:
    target = frac * t_dur
    # frame raw más cercano
    f_raw = None
    if len(data['raw']['t']) > 0:
        idx = int(np.argmin(np.abs(data['raw']['t'] - target)))
        f_raw = data['raw']['v'][idx]
    # frame debug más cercano
    f_dbg = None
    if len(data['dbg']['t']) > 0:
        idx = int(np.argmin(np.abs(data['dbg']['t'] - target)))
        f_dbg = data['dbg']['v'][idx]
    snaps.append((target, f_raw, f_dbg))


# ── Figura ───────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 14))
fig.suptitle(
    "CapyTown — Visualización Lane Following  |  RC-2 ESAN 2026-I",
    fontsize=14, fontweight='bold', y=0.99)

# GridSpec: 4 filas
# Fila 0: error (ancho total)
# Fila 1: yellow_x (mitad izq) + white_x (mitad der)
# Filas 2-3: 4 snapshots × 2 imágenes (raw | debug)
gs = gridspec.GridSpec(4, 4, figure=fig, hspace=0.52, wspace=0.30,
                       height_ratios=[1.8, 1.4, 2, 2])

# ── Fila 0: Error lateral ────────────────────────────────────────────────────
ax_e = fig.add_subplot(gs[0, :])
te, ev = data['err']['t'], data['err']['v']
valid  = ~np.isnan(ev)

ax_e.plot(te, ev, color='#90CAF9', linewidth=0.6, alpha=0.5)
if valid.sum() > 0:
    ax_e.plot(te[valid], ev[valid], color='#1565C0', linewidth=1.2, label='error lateral')
    ax_e.fill_between(te[valid], ev[valid], 0,
                      where=(ev[valid] > 0), alpha=0.13, color='#E53935', label='drift→derecha')
    ax_e.fill_between(te[valid], ev[valid], 0,
                      where=(ev[valid] < 0), alpha=0.13, color='#1E88E5', label='drift→izquierda')
    mu = ev[valid].mean()
    ax_e.axhline(mu, color='orange', linewidth=0.9, linestyle='-.', label=f'μ={mu:.4f} m')

ax_e.axhline(0, color='#43A047', linewidth=1.0, linestyle='--', label='setpoint')
ax_e.axhspan(-0.02, 0.02, alpha=0.07, color='green', label='±0.02 m')

for t_s, _, _ in snaps:
    ax_e.axvline(t_s, color='orange', linewidth=0.9, linestyle=':', alpha=0.7)

ax_e.set_xlim(0, t_dur); ax_e.set_ylabel('error (m)', fontsize=10)
ax_e.set_title('/lane_error — Error lateral en el tiempo', fontsize=11)
ax_e.legend(loc='upper right', fontsize=7.5, ncol=4)
ax_e.grid(True, alpha=0.3)

# ── Fila 1 izq: Posición X amarillo ──────────────────────────────────────────
ax_y = fig.add_subplot(gs[1, :2])
ty, yv = data['yx']['t'], data['yx']['v']
valid_y = ~np.isnan(yv) if len(yv) > 0 else np.array([], dtype=bool)

if valid_y.sum() > 0:
    ax_y.scatter(ty[valid_y], yv[valid_y], s=1.5, color='#F9A825', alpha=0.7, label='yellow_x')
    ax_y.plot(ty[valid_y], yv[valid_y], color='#F9A825', linewidth=0.8, alpha=0.5)

if len(ty) > 0 and (~valid_y).sum() > 0:
    ax_y.scatter(ty[~valid_y], np.zeros(( ~valid_y).sum()), s=3,
                 color='red', alpha=0.4, marker='x', label='no detectado')

ax_y.axhline(0.33, color='gray', linewidth=0.8, linestyle='--', label='setpoint 0.33')
for t_s, _, _ in snaps:
    ax_y.axvline(t_s, color='orange', linewidth=0.9, linestyle=':', alpha=0.7)

ax_y.set_xlim(0, t_dur); ax_y.set_ylim(-0.05, 1.05)
ax_y.set_ylabel('x / ancho imagen', fontsize=9)
ax_y.set_title('/lane/yellow_x — Posición línea amarilla', fontsize=10)
ax_y.legend(loc='upper right', fontsize=7.5)
ax_y.grid(True, alpha=0.3)

if valid_y.sum() > 5:
    stats = (f"μ={yv[valid_y].mean():.3f}  σ={yv[valid_y].std():.3f}\n"
             f"detectado {100*valid_y.mean():.1f}%")
    ax_y.text(0.02, 0.95, stats, transform=ax_y.transAxes,
              fontsize=8, va='top', bbox=dict(boxstyle='round', fc='#FFFDE7', alpha=0.8))

# ── Fila 1 der: Posición X blanco ─────────────────────────────────────────────
ax_w = fig.add_subplot(gs[1, 2:])
tw, wv = data['wx']['t'], data['wx']['v']
valid_w = ~np.isnan(wv) if len(wv) > 0 else np.array([], dtype=bool)

if valid_w.sum() > 0:
    ax_w.scatter(tw[valid_w], wv[valid_w], s=1.5, color='#78909C', alpha=0.7, label='white_x')
    ax_w.plot(tw[valid_w], wv[valid_w], color='#546E7A', linewidth=0.8, alpha=0.5)

if len(tw) > 0 and (~valid_w).sum() > 0:
    ax_w.scatter(tw[~valid_w], np.zeros((~valid_w).sum()), s=3,
                 color='red', alpha=0.4, marker='x', label='no detectado (curva)')

ax_w.axhline(1.0 - 0.33, color='gray', linewidth=0.8, linestyle='--', label='setpoint 0.67')
for t_s, _, _ in snaps:
    ax_w.axvline(t_s, color='orange', linewidth=0.9, linestyle=':', alpha=0.7)

ax_w.set_xlim(0, t_dur); ax_w.set_ylim(-0.05, 1.05)
ax_w.set_ylabel('x / ancho imagen', fontsize=9)
ax_w.set_title('/lane/white_x — Posición línea blanca', fontsize=10)
ax_w.legend(loc='upper right', fontsize=7.5)
ax_w.grid(True, alpha=0.3)

if valid_w.sum() > 5:
    stats = (f"μ={wv[valid_w].mean():.3f}  σ={wv[valid_w].std():.3f}\n"
             f"detectado {100*valid_w.mean():.1f}%")
    ax_w.text(0.02, 0.95, stats, transform=ax_w.transAxes,
              fontsize=8, va='top', bbox=dict(boxstyle='round', fc='#ECEFF1', alpha=0.8))

# ── Filas 2-3: 4 snapshots (raw izq | debug der) ────────────────────────────
PLACEHOLDER = np.full((240, 320, 3), 30, dtype=np.uint8)

for col, (t_s, f_raw, f_dbg) in enumerate(snaps):
    row_offset = 2 if col < 2 else 3
    col_offset = (col % 2) * 2

    # cámara normal
    ax_r = fig.add_subplot(gs[row_offset, col_offset])
    img_r = cv2.cvtColor(f_raw if f_raw is not None else PLACEHOLDER, cv2.COLOR_BGR2RGB)
    ax_r.imshow(img_r)
    ax_r.set_title(f'{LABELS[col]}\nCámara normal  t={t_s:.1f}s', fontsize=8)
    ax_r.axis('off')

    # cámara con detección
    ax_d = fig.add_subplot(gs[row_offset, col_offset + 1])
    img_d = cv2.cvtColor(f_dbg if f_dbg is not None else PLACEHOLDER, cv2.COLOR_BGR2RGB)
    ax_d.imshow(img_d)
    ax_d.set_title(f'{LABELS[col]}\nCon detección  t={t_s:.1f}s', fontsize=8)
    ax_d.axis('off')

    # Si no hay raw, anotar
    if f_raw is None:
        ax_r.text(0.5, 0.5, 'Sin /camera/image_raw\nen el bag',
                  transform=ax_r.transAxes, ha='center', va='center',
                  color='white', fontsize=9)
    if f_dbg is None:
        ax_d.text(0.5, 0.5, 'Sin /lane/debug_image\nen el bag',
                  transform=ax_d.transAxes, ha='center', va='center',
                  color='white', fontsize=9)

plt.savefig(out_file, dpi=150, bbox_inches='tight')
print(f"\nVisualizacion guardada: {os.path.abspath(out_file)}")

# ── Estadísticas finales en consola ──────────────────────────────────────────
if valid.sum() > 0:
    print(f"\n── Estadísticas /lane_error ──")
    print(f"  Media:    {ev[valid].mean():.5f} m")
    print(f"  Std:      {ev[valid].std():.5f} m")
    print(f"  RMS:      {np.sqrt((ev[valid]**2).mean()):.5f} m")
    print(f"  Max abs:  {np.abs(ev[valid]).max():.5f} m")
    print(f"  Cobertura: {100*valid.mean():.1f}%")
if valid_y.sum() > 0:
    print(f"\n── Detección amarillo ──")
    print(f"  Detectado: {100*valid_y.mean():.1f}% del tiempo")
    print(f"  Posición media: {yv[valid_y].mean():.3f} (setpoint 0.33)")
if valid_w.sum() > 0:
    print(f"\n── Detección blanco ──")
    print(f"  Detectado: {100*valid_w.mean():.1f}% del tiempo  (normal perder en curvas)")
    print(f"  Posición media: {wv[valid_w].mean():.3f} (setpoint 0.67)")