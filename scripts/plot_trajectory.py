#!/usr/bin/env python3
"""
Genera plot de trayectoria XY desde /odom_raw + coloreado por error lateral.

Produce trajectory_s11.png con:
  - Trayectoria real (3 vueltas en colores distintos)
  - Track ideal superpuesto
  - Error de cierre (distancia inicio → fin)
  - Panel lateral: /lane_error vs tiempo

Uso:
    python3 scripts/plot_trajectory.py <ruta_al_bag>
    python3 scripts/plot_trajectory.py ~/bags/lane_s11_traj
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import LineCollection
import matplotlib.gridspec as gridspec

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float32
    from nav_msgs.msg import Odometry
except ImportError:
    print("ERROR: ejecuta dentro del entorno ROS2:")
    print("       source /opt/ros/humble/setup.bash")
    sys.exit(1)

if len(sys.argv) < 2:
    print(f"Uso: python3 {sys.argv[0]} <ruta_bag> [salida.png]")
    sys.exit(1)

bag_path = sys.argv[1]
out_file  = sys.argv[2] if len(sys.argv) > 2 else "trajectory_s11.png"

if not os.path.exists(bag_path):
    print(f"ERROR: no existe '{bag_path}'")
    sys.exit(1)

# ── Leer bag ─────────────────────────────────────────────────────────────────
reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions('', ''),
)

t0 = None
odom_t, odom_x, odom_y, odom_yaw = [], [], [], []
err_t,  err_v = [], []

import math

while reader.has_next():
    topic, data, stamp = reader.read_next()
    if t0 is None:
        t0 = stamp
    t = (stamp - t0) * 1e-9

    if topic == '/odom_raw':
        msg = deserialize_message(data, Odometry)
        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        # yaw desde quaternion
        q   = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        odom_t.append(t); odom_x.append(px); odom_y.append(py); odom_yaw.append(yaw)

    elif topic == '/lane_error':
        msg = deserialize_message(data, Float32)
        err_t.append(t); err_v.append(msg.data)

if not odom_x:
    print("ERROR: No hay datos de /odom_raw en el bag.")
    sys.exit(1)

odom_x   = np.array(odom_x)
odom_y   = np.array(odom_y)
odom_t   = np.array(odom_t)
err_t    = np.array(err_t)
err_v    = np.array(err_v)
valid_e  = ~np.isnan(err_v)

t_dur = odom_t[-1]
print(f"Duración: {t_dur:.1f}s | Puntos odom: {len(odom_x)} | Errores: {len(err_v)}")

# ── Interpolar error en los puntos de odometría ───────────────────────────────
if len(err_t) > 1:
    err_interp = np.interp(odom_t, err_t, np.where(np.isnan(err_v), 0, err_v))
    err_abs    = np.abs(err_interp)
else:
    err_abs = np.zeros(len(odom_t))

# ── Dividir en 3 vueltas por tiempo ──────────────────────────────────────────
lap_colors = ['#E53935', '#FB8C00', '#1E88E5']  # rojo, naranja, azul
lap_labels = ['Vuelta 1', 'Vuelta 2', 'Vuelta 3']
lap_masks  = []
for i in range(3):
    t_lo = i * t_dur / 3
    t_hi = (i + 1) * t_dur / 3
    lap_masks.append((odom_t >= t_lo) & (odom_t < t_hi))

# ── Error de cierre ───────────────────────────────────────────────────────────
closure_err = math.hypot(odom_x[-1] - odom_x[0], odom_y[-1] - odom_y[0])
print(f"Error de cierre: {closure_err*100:.1f} cm")

# ── Estimar forma del track (bounding box + margen) ──────────────────────────
x_min, x_max = odom_x.min(), odom_x.max()
y_min, y_max = odom_y.min(), odom_y.max()
W = x_max - x_min
H = y_max - y_min
cx = (x_min + x_max) / 2
cy = (y_min + y_max) / 2

def rounded_rect(cx, cy, w, h, r=0.08, n=80):
    """Genera los puntos de un rectángulo con esquinas redondeadas."""
    pts = []
    corners = [
        (cx - w/2 + r, cy - h/2 + r, math.pi,   3*math.pi/2),
        (cx + w/2 - r, cy - h/2 + r, 3*math.pi/2, 2*math.pi),
        (cx + w/2 - r, cy + h/2 - r, 0,          math.pi/2),
        (cx - w/2 + r, cy + h/2 - r, math.pi/2,  math.pi),
    ]
    for (ox, oy, a0, a1) in corners:
        for a in np.linspace(a0, a1, n // 4):
            pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    pts.append(pts[0])
    return np.array(pts)

ideal_outer = rounded_rect(cx, cy, W * 1.05, H * 1.05, r=min(W, H) * 0.08)
ideal_inner = rounded_rect(cx, cy, W * 0.65, H * 0.65, r=min(W, H) * 0.06)
ideal_mid   = rounded_rect(cx, cy, W * 0.85, H * 0.85, r=min(W, H) * 0.07)

# ── Figura ────────────────────────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 7))
fig.suptitle(f"Trayectoria XY — Error de cierre: {closure_err*100:.1f} cm  |  CapyTown RC-2 ESAN 2026-I",
             fontsize=13, fontweight='bold')

gs = gridspec.GridSpec(1, 2, figure=fig, wspace=0.35, width_ratios=[1.2, 1])

# ── Panel izquierdo: trayectoria XY ──────────────────────────────────────────
ax = fig.add_subplot(gs[0])

# Track ideal
ax.plot(ideal_outer[:, 0], ideal_outer[:, 1], '--', color='white',  linewidth=2.5, alpha=0.9, label='Línea blanca (ideal)')
ax.plot(ideal_inner[:, 0], ideal_inner[:, 1], '--', color='#F9A825', linewidth=2.5, alpha=0.9, label='Línea amarilla (ideal)')
ax.plot(ideal_mid[:, 0],   ideal_mid[:, 1],   '--', color='#555555', linewidth=1.0, alpha=0.4, label='Centro ideal')
ax.set_facecolor('#1a1a2e')

# Trayectoria por vueltas
for i, (mask, color, label) in enumerate(zip(lap_masks, lap_colors, lap_labels)):
    if mask.sum() > 1:
        ax.plot(odom_x[mask], odom_y[mask], color=color, linewidth=1.8,
                alpha=0.85, label=label, zorder=3)

# Inicio y fin
ax.scatter(odom_x[0],  odom_y[0],  s=120, c='#00E676', zorder=5,
           marker='o', label=f'Inicio')
ax.scatter(odom_x[-1], odom_y[-1], s=120, c='#FF1744', zorder=5,
           marker='s', label=f'Fin  (Δ={closure_err*100:.1f} cm)')

# Flecha de dirección en el inicio
if len(odom_yaw) > 0:
    dy = math.sin(odom_yaw[0]) * 0.04
    dx = math.cos(odom_yaw[0]) * 0.04
    ax.annotate('', xy=(odom_x[0]+dx, odom_y[0]+dy),
                xytext=(odom_x[0], odom_y[0]),
                arrowprops=dict(arrowstyle='->', color='#00E676', lw=2))

ax.set_xlabel('x (m)', fontsize=11)
ax.set_ylabel('y (m)', fontsize=11)
ax.set_title('Trayectoria real — 3 vueltas', fontsize=11)
ax.set_aspect('equal')
ax.legend(loc='upper right', fontsize=7.5, facecolor='#2a2a3e', labelcolor='white')
ax.tick_params(colors='white')
ax.xaxis.label.set_color('white'); ax.yaxis.label.set_color('white')
ax.title.set_color('white')
for spine in ax.spines.values():
    spine.set_edgecolor('#555')

# ── Panel derecho: error vs tiempo con lap markers ────────────────────────────
ax2 = fig.add_subplot(gs[1])

for i, (color, label) in enumerate(zip(lap_colors, lap_labels)):
    t_lo = i * t_dur / 3
    t_hi = (i + 1) * t_dur / 3
    mask = (err_t >= t_lo) & (err_t < t_hi)
    if valid_e[mask].sum() > 0:
        ax2.plot(err_t[mask & valid_e], err_v[mask & valid_e],
                 color=color, linewidth=0.9, alpha=0.85, label=label)
    ax2.axvline(t_lo, color=color, linewidth=0.7, linestyle=':', alpha=0.5)

ax2.axhline(0,     color='#43A047', linewidth=1.0, linestyle='--', label='Setpoint')
ax2.axhspan(-0.02, 0.02, alpha=0.08, color='green', label='±2 cm')

if valid_e.sum() > 0:
    ev = err_v[valid_e]
    mu = ev.mean()
    ax2.axhline(mu, color='orange', linewidth=0.8, linestyle='-.',
                label=f'μ={mu:.3f} m')
    stats = (f"Media:  {mu:.4f} m\n"
             f"RMS:    {np.sqrt((ev**2).mean()):.4f} m\n"
             f"Máx:    {np.abs(ev).max():.4f} m\n"
             f"Cobert: {100*valid_e.mean():.1f}%")
    ax2.text(0.02, 0.97, stats, transform=ax2.transAxes,
             fontsize=8, va='top',
             bbox=dict(boxstyle='round', facecolor='#F5F5F5', alpha=0.85))

ax2.set_xlim(0, t_dur)
ax2.set_xlabel('tiempo (s)', fontsize=10)
ax2.set_ylabel('error lateral (m)', fontsize=10)
ax2.set_title('/lane_error — 3 vueltas', fontsize=10)
ax2.legend(loc='lower right', fontsize=7.5)
ax2.grid(True, alpha=0.3)

plt.savefig(out_file, dpi=150, bbox_inches='tight', facecolor='#f8f8f8')
print(f"\nTrayectoria guardada: {os.path.abspath(out_file)}")