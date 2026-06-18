#!/usr/bin/env python3
"""
Genera reporte visual completo desde un ros2 bag.

Produce lane_report_s11.png con:
  - Error lateral vs tiempo (corrida completa)
  - Estado de detección Y / W / NONE a lo largo del tiempo
  - Histograma del error lateral
  - 4 fotogramas de la cámara de debug en momentos distintos de la corrida

Uso:
    python3 scripts/lane_report.py <ruta_al_bag>
    python3 scripts/lane_report.py ~/bags/lane_error_s11
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
out_file  = sys.argv[2] if len(sys.argv) > 2 else "lane_report_s11.png"

if not os.path.exists(bag_path):
    print(f"ERROR: no existe '{bag_path}'")
    sys.exit(1)

# ── Leer bag ─────────────────────────────────────────────────────────────────
bridge = CvBridge()

reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions('', ''),
)

t0 = None
ts_err, errs = [], []
ts_img, imgs = [], []

while reader.has_next():
    topic, data, stamp = reader.read_next()
    if t0 is None:
        t0 = stamp
    t = (stamp - t0) * 1e-9

    if topic == '/lane_error':
        msg = deserialize_message(data, Float32)
        ts_err.append(t)
        errs.append(msg.data)

    elif topic == '/lane/debug_image':
        msg = deserialize_message(data, Image)
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ts_img.append(t)
        imgs.append(frame)

if not ts_err:
    print("ERROR: No hay mensajes /lane_error en el bag.")
    sys.exit(1)

ts_err = np.array(ts_err)
errs   = np.array(errs)
valid  = ~np.isnan(errs)
t_dur  = ts_err[-1]
print(f"Duración: {t_dur:.1f}s | Errores: {len(ts_err)} | Frames: {len(imgs)}")

# ── Seleccionar 4 fotogramas en cuartos de la corrida ────────────────────────
snap_frames = []
if imgs:
    checkpoints = [0.15, 0.38, 0.62, 0.85]
    for frac in checkpoints:
        target_t = frac * t_dur
        idx = int(np.argmin(np.abs(np.array(ts_img) - target_t)))
        snap_frames.append((ts_img[idx], imgs[idx]))

# ── Figura ───────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 11))
fig.suptitle("CapyTown — Reporte Lane Following  |  RC-2 ESAN 2026-I",
             fontsize=14, fontweight='bold', y=0.98)

gs = gridspec.GridSpec(3, 4, figure=fig,
                       hspace=0.45, wspace=0.35,
                       height_ratios=[2.5, 0.6, 2.2])

# ── Fila 0: error vs tiempo (ocupa las 4 columnas) ──────────────────────────
ax_err = fig.add_subplot(gs[0, :])
ax_err.plot(ts_err, errs, color='#90CAF9', linewidth=0.7, alpha=0.7)
ax_err.plot(ts_err[valid], errs[valid], color='#1565C0', linewidth=1.1, label='error lateral')
ax_err.axhline(0, linestyle='--', color='#43A047', linewidth=0.9, label='setpoint (0)')
ax_err.fill_between(ts_err[valid], errs[valid], 0,
                    where=(errs[valid] > 0), alpha=0.12, color='#E53935', label='drift→derecha')
ax_err.fill_between(ts_err[valid], errs[valid], 0,
                    where=(errs[valid] < 0), alpha=0.12, color='#1E88E5', label='drift→izquierda')
ax_err.axhspan(-0.02, 0.02, alpha=0.07, color='green', label='±0.02 m tolerancia')

# Marcar los 4 snapshots
for t_snap, _ in snap_frames:
    ax_err.axvline(t_snap, color='orange', linewidth=1.0, linestyle=':', alpha=0.8)

if valid.sum() > 5:
    mu = errs[valid].mean()
    ax_err.axhline(mu, color='orange', linewidth=0.8, linestyle='-.',
                   label=f'media={mu:.4f} m')

ax_err.set_xlabel('tiempo (s)', fontsize=10)
ax_err.set_ylabel('error (m)', fontsize=10)
ax_err.set_title('Error lateral /lane_error — corrida completa', fontsize=11)
ax_err.legend(loc='upper right', fontsize=7.5, ncol=3)
ax_err.grid(True, alpha=0.3)
ax_err.set_xlim(left=0, right=t_dur)

# ── Fila 1: barra de estado de detección (4 columnas) ───────────────────────
ax_st = fig.add_subplot(gs[1, :])
ax_st.fill_between(ts_err, 0, 1,
                   where=valid,  color='#43A047', alpha=0.7, label='línea detectada (Y o W)')
ax_st.fill_between(ts_err, 0, 1,
                   where=~valid, color='#E53935', alpha=0.7, label='sin línea → parado')
for t_snap, _ in snap_frames:
    ax_st.axvline(t_snap, color='orange', linewidth=1.5, linestyle=':', alpha=0.9)
ax_st.set_xlim(left=0, right=t_dur)
ax_st.set_ylim(0, 1)
ax_st.set_yticks([])
ax_st.set_xlabel('tiempo (s)', fontsize=9)
ax_st.set_title('Estado de detección', fontsize=9)
ax_st.legend(loc='upper right', fontsize=7.5)

# ── Fila 2 col 0: histograma ─────────────────────────────────────────────────
ax_hist = fig.add_subplot(gs[2, 0])
if valid.sum() > 5:
    ev = errs[valid]
    ax_hist.hist(ev, bins=40, color='#1565C0', alpha=0.75, edgecolor='white', linewidth=0.4)
    ax_hist.axvline(0,        color='green',  linewidth=1.2, linestyle='--')
    ax_hist.axvline(ev.mean(), color='orange', linewidth=1.2, linestyle='-.',
                    label=f'μ={ev.mean():.4f}')
    ax_hist.set_xlabel('error (m)', fontsize=9)
    ax_hist.set_ylabel('frecuencia', fontsize=9)
    ax_hist.set_title('Histograma del error', fontsize=9)
    ax_hist.legend(fontsize=8)
    ax_hist.grid(True, alpha=0.3)

    stats_text = (
        f"Muestras válidas: {valid.sum()}/{len(errs)}\n"
        f"Cobertura:  {100*valid.mean():.1f}%\n"
        f"Media:      {ev.mean():.5f} m\n"
        f"Std:        {ev.std():.5f} m\n"
        f"Max:        {ev.max():.5f} m\n"
        f"Min:        {ev.min():.5f} m\n"
        f"RMS:        {np.sqrt((ev**2).mean()):.5f} m\n"
        f"Duración:   {t_dur:.1f} s"
    )
    ax_hist.text(0.98, 0.97, stats_text,
                 transform=ax_hist.transAxes,
                 fontsize=7, verticalalignment='top', horizontalalignment='right',
                 bbox=dict(boxstyle='round', facecolor='#F5F5F5', alpha=0.8))

# ── Fila 2 cols 1-3: 4 fotogramas de cámara ──────────────────────────────────
labels_pos = ['t≈15%', 't≈38%', 't≈62%', 't≈85%']
axes_img = [fig.add_subplot(gs[2, c]) for c in range(1, 4)]

for col_idx in range(3):
    ax_img = axes_img[col_idx]
    snap_idx = col_idx + 1  # snapshots 1,2,3 (4 totales, dejamos la 0 si no hay columna)
    if snap_idx < len(snap_frames):
        t_snap, frame = snap_frames[snap_idx]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ax_img.imshow(rgb)
        ax_img.set_title(f'Frame @ {t_snap:.1f}s  ({labels_pos[snap_idx]})',
                         fontsize=8)
    ax_img.axis('off')

# Primer snapshot en ax_hist si hay imágenes
if snap_frames:
    t_snap0, frame0 = snap_frames[0]
    ax_first = fig.add_axes([0.255, 0.01, 0.14, 0.25])  # posición manual
    rgb0 = cv2.cvtColor(frame0, cv2.COLOR_BGR2RGB)
    ax_first.imshow(rgb0)
    ax_first.set_title(f'Frame @ {t_snap0:.1f}s  ({labels_pos[0]})', fontsize=8)
    ax_first.axis('off')

plt.savefig(out_file, dpi=150, bbox_inches='tight')
print(f"\nReporte guardado en: {os.path.abspath(out_file)}")