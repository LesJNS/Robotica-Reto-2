# Próximos pasos — RC-2 (Las 3 Vueltas del Jirón)

El código ya quedó corregido y optimizado (ver sección "Qué se cambió" al final). Lo que
falta ahora es **operativo, no de código**: correr el robot, grabar evidencia y subirla.
Sigue estos pasos en orden.

## 1. Recompilar el paquete

Cambiamos el nombre de un tópico (`/image_raw` → `/camera/image_raw`) en todos los nodos
a la vez, así que no rompe nada — pero hay que recompilar antes de lanzar:

```bash
cd ~/ros2_ws        # ajusta a la ruta real de tu workspace
colcon build --packages-select capytown_esan
source install/setup.bash
```

## 2. Verificar que los tópicos están vivos

```bash
ros2 launch capytown_esan lane_following.launch.py
```

En otra terminal (con el mismo `source install/setup.bash`):

```bash
ros2 topic list                              # deben aparecer /camera/image_raw, /lane_error, /cmd_vel, etc.
ros2 topic hz /camera/image_raw              # ¿llega a ~30 Hz?
ros2 topic hz /lane_error                    # ¿llega a ~30 Hz?
```

## 3. Verificar la IPM visualmente

Antes solo podías verificar `/lane/debug_image`. Ahora `lane_detector.py` también
publica `/lane/raw_trapezoid` y `/lane/birdeye_image`, así que el visor web
(`camera_stream`) ya muestra los 3 paneles correctamente:

```
http://<IP_DEL_ROBOT>:5800/
```

* Panel 1 (`raw_trapezoid`): el cuadrilátero cyan debe cubrir ambas líneas del carril.
* Panel 2 (`birdeye_image`): las líneas deben verse rectas y verticales. Si se ven
  inclinadas o curvas, recalibra los puntos de `build_ipm()` en `lane_detector.py`.
* Panel 3 (`debug_image`): máscaras de color + centroides, como antes.

Si algo no se ve bien, **recalibra HSV con la luz real del día**:

```bash
export DISPLAY=:0
ros2 run capytown_esan hsv_tuner
# ajustar trackbars → 's' guarda en hsv_tuner_output.yaml → copiar valores a config/hsv_params.yaml
```

## 4. Sintonizar PID (si aún no quedó conforme)

Un parámetro por prueba, un bag por cambio (regla de la sesión). Edita
`config/pid_params.yaml`, relanza, y registra:

```bash
ros2 bag record /lane_error /cmd_vel /camera/image_raw /lane/debug_image -o s11_prueba_N
```

Orden de sintonización: `kp` → `kd` → `ki`. Confirma que `linear_speed ≥ 0.20` (ya está
en 0.30 en el yaml).

## 5. Grabar la corrida final — las 3 vueltas

Cuando el sistema esté sintonizado, graba la corrida que vas a entregar:

```bash
ros2 bag record /lane_error /cmd_vel /camera/image_raw /lane/debug_image \
    /lane/yellow_x /lane/white_x -o s11_final
```

Deja correr el robot 3 vueltas completas sin intervención manual. Si se sale del
jirón o necesitas tocarlo, descarta el bag y repite.

> Nota: ningún nodo de este paquete publica odometría (`/odom`); si tu robot expone un
> tópico de odometría real (driver Yahboom), agrégalo a la grabación y ajusta el nombre
> en `scripts/plot_trajectory.py:66` (hoy asume `/odom_raw`) antes de usarlo.

## 6. Generar la evidencia desde el bag

Desde la raíz del repo, con el entorno ROS2 activo:

```bash
python3 scripts/plot_lane_error.py s11_final          # → lane_error_s11.png
python3 scripts/lane_report.py s11_final               # → lane_report_s11.png
python3 scripts/lane_visualization.py s11_final         # → lane_visualization_s11.png
# solo si grabaste odometría real:
python3 scripts/plot_trajectory.py s11_final            # → trajectory_s11.png
```

Revisa en consola que `Error medio` quede **≤ 3 cm** (criterio de la rúbrica). Si no,
vuelve al paso 4.

## 7. Grabar el video MP4

Grábate la pantalla del visor web (`http://<IP>:5800/`) o la pista físicamente durante
las 3 vueltas. Guarda el archivo como `video_s11.mp4` en la raíz del repo (no está
en `.gitignore`, así que se puede commitear directo).

## 8. (Opcional) Bonus IPM

El enunciado pide comparar el error con y sin IPM. Hoy no hay forma de desactivar la
IPM sin tocar código. Si quieres el punto bonus:

1. Añade un parámetro `use_ipm` (bool) a `lane_detector.py` que, si es `false`, use
   `frame` directamente en vez de `warp` para las máscaras y centroides.
2. Graba un bag corto con `use_ipm:=false` y otro con `use_ipm:=true` en el mismo tramo
   de pista.
3. Compara el error medio de ambos con `scripts/plot_lane_error.py`.

## 9. Commitear evidencia y crear el tag

```bash
git add lane_error_s11.png lane_report_s11.png lane_visualization_s11.png video_s11.mp4
# si grabaste odometría real:
git add trajectory_s11.png
git add config/pid_params.yaml config/hsv_params.yaml   # valores finales de sintonización
git commit -m "RC-2: evidencia final — 3 vueltas, PID sintonizado"
git tag s11
git push origin <tu-rama> --tags
```

## 10. Checklist final contra la rúbrica

- [ ] 3 vueltas completas sin salirse del jirón (video o corrida presencial)
- [ ] Velocidad promedio ≥ 0.2 m/s (verificar en consola de `plot_lane_error.py`, no solo en el yaml)
- [ ] `\|/lane_error\|` medio ≤ 3 cm (consola de `plot_lane_error.py`)
- [ ] `lane_error_s11.png` commiteado
- [ ] Video MP4 commiteado
- [ ] Tag `s11` creado y empujado
- [ ] (Bonus) comparativa con/sin IPM, si decides implementarla

---

## Qué se cambió en el código (resumen)

| Archivo | Cambio | Por qué |
|---|---|---|
| `capytown_esan/lane_controller.py` | `control_loop()` ahora frena si no llega un `/lane_error` fresco en más de `error_timeout` segundos (antes solo reaccionaba a NaN explícito) | El enunciado exige frenar también si el sensor "cae" (tópico mudo), no solo ante NaN |
| `capytown_esan/lane_controller.py`, `config/pid_params.yaml` | Se eliminaron `recovery_w`/`recovery_v` (parámetros declarados pero nunca usados) | Código muerto que no afectaba el comportamiento real (el robot ya frenaba, no "buscaba") |
| `capytown_esan/cam_pub.py`, `lane_detector.py`, `camera_stream.py`, `hsv_tuner.py`, `scripts/lane_visualization.py` | Tópico de cámara renombrado de `/image_raw` a `/camera/image_raw` | Así coincide con la tabla de tópicos del Bloque 3 del enunciado |
| `capytown_esan/lane_detector.py` | Nuevos publishers `/lane/raw_trapezoid` y `/lane/birdeye_image` | `camera_stream.py` ya se suscribía a esos tópicos pero nadie los publicaba — los paneles 1 y 2 del visor web estaban rotos |
| `config/pid_params.yaml` | Comentario `TURBO TEST 🚀` reemplazado por una nota explicando el requisito de velocidad mínima | Limpieza — el valor numérico (0.30) no cambió |
| `.gitignore` | Excepciones para `lane_error_s11.png`, `trajectory_s11.png`, `lane_report_s11.png`, `lane_visualization_s11.png` | Antes el repo ignoraba `*.png` globalmente, lo que impedía commitear la evidencia que la rúbrica exige |

No se modificó la lógica de PID, HSV, IPM, anti-windup, ni el conteo de vueltas — esa
parte ya cumplía con el enunciado.
