#!/usr/bin/env python3
"""Genera el plot de /lane_error vs. tiempo desde un ros2 bag.

Uso:
    python3 scripts/plot_lane_error.py <ruta_al_bag>

Ejemplo:
    python3 scripts/plot_lane_error.py ~/bags/s11_kp40

Salida:
    lane_error_s11.png  (en el directorio actual)
"""

import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')   # sin pantalla — no llama a plt.show()
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float32
except ImportError:
    print("ERROR: Este script debe ejecutarse en el entorno ROS2.")
    print("       source /opt/ros/humble/setup.bash")
    sys.exit(1)

if len(sys.argv) < 2:
    print(f"Uso: python3 {sys.argv[0]} <ruta_bag>")
    sys.exit(1)

bag_path = sys.argv[1]
if not os.path.exists(bag_path):
    print(f"ERROR: No se encuentra el bag en '{bag_path}'")
    sys.exit(1)

# --- Leer el bag ---
reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions('', ''),
)

t0, ts, errs = None, [], []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if topic == '/lane_error':
        if t0 is None:
            t0 = stamp
        ts.append((stamp - t0) * 1e-9)
        errs.append(deserialize_message(data, Float32).data)

if not ts:
    print("ERROR: No se encontraron mensajes de /lane_error en el bag.")
    sys.exit(1)

ts   = np.array(ts)
errs = np.array(errs)
valid = ~np.isnan(errs)
ev    = errs[valid]

print(f"Muestras leídas:  {len(ts)}")
print(f"Duración total:   {ts[-1]:.1f} s")
print(f"Muestras válidas: {valid.sum()} ({100*valid.mean():.1f}%)")
if len(ev) > 0:
    print(f"Error medio:      {ev.mean():.4f} m  ({ev.mean()*100:.1f} cm)")
    print(f"Error std:        {ev.std():.4f} m")
    print(f"Error RMS:        {np.sqrt((ev**2).mean()):.4f} m")
    print(f"Error máx abs:    {np.abs(ev).max():.4f} m  ({np.abs(ev).max()*100:.1f} cm)")

# --- Plot ---
fig, ax = plt.subplots(figsize=(12, 4))

ax.plot(ts, errs, color='royalblue', linewidth=0.9, label='/lane_error')
ax.axhline(0, linestyle='--', color='gray', linewidth=0.8)
ax.fill_between(ts[valid], errs[valid], 0, alpha=0.15, color='royalblue')

# Banda de tolerancia ±0.02 m
ax.axhspan(-0.02, 0.02, alpha=0.08, color='green')
tol_patch = mpatches.Patch(color='green', alpha=0.3, label='tolerancia ±0.02 m')

if len(ev) > 0:
    ax.axhline(ev.mean(), color='orange', linewidth=0.9, linestyle='-.',
               label=f'media={ev.mean():.4f} m')

# Estadísticas en el plot
stats = (f"Media: {ev.mean():.4f} m\n"
         f"RMS:   {np.sqrt((ev**2).mean()):.4f} m\n"
         f"Máx:   {np.abs(ev).max():.4f} m\n"
         f"Cob.:  {100*valid.mean():.1f}%") if len(ev) > 0 else ""
ax.text(0.01, 0.97, stats, transform=ax.transAxes,
        fontsize=8, verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='#F5F5F5', alpha=0.8))

ax.set_xlabel('tiempo (s)', fontsize=11)
ax.set_ylabel('/lane_error  (m)', fontsize=11)
ax.set_title('Error lateral — 3 vueltas RC-2  |  CapyTown ESAN 2026-I', fontsize=12)
ax.legend(fontsize=9)
ax.grid(True, alpha=0.4)
ax.set_xlim(left=0)

plt.tight_layout()
out_file = 'lane_error_s11.png'
plt.savefig(out_file, dpi=150)
print(f"\nPlot guardado en: {os.path.abspath(out_file)}")