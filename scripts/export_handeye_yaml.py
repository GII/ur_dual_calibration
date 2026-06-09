#!/usr/bin/env python3
"""
Exporta el resultado de calibrateHandEye (PARK) componiéndolo con la
transformación interna de la OAK-D, para publicar el TF estático sin
conflictos de doble padre.

Flujo:
  1. Lee handeye_results.json → T(ur_I_base → oak_rgb_optical).
  2. Consulta el driver OAK en vivo → T(oak_rgb_optical → oak-d-base-frame).
  3. Compone las dos → T(ur_I_base → oak-d-base-frame).
  4. Escribe handeye.yaml con parent=ur_I_base y child=oak-d-base-frame.

Requiere que la OAK-D esté publicando su árbol TF interno antes de correr
este script (lanza el driver OAK antes). NO debe haber un
static_transform_publisher nuestro activo (Ctrl+C cualquier launch previo).
"""
import json
import numpy as np
import rclpy
import sys
import time
from pathlib import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parent
IN_PATH    = REPO_ROOT / 'calibration_data' / 'results' / 'handeye_results.json'
OUT_PATH   = REPO_ROOT / 'src' / 'ur_dual_calibration' / 'config' / 'handeye.yaml'

METHOD = 'PARK'

# Frame raíz de la isla OAK (el que no tiene padre dentro de su propio árbol).
# Se conecta el árbol del robot a ESTE frame, no al optical, para evitar el
# conflicto de doble padre (el optical ya es hijo de oak_rgb_camera_frame).
OAK_ROOT_FRAME    = 'oak-d-base-frame'
OAK_OPTICAL_FRAME = 'oak_rgb_camera_optical_frame'


def quat_to_R(q):
    x, y, z, w = q
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-z*w),    2*(x*z+y*w)],
        [2*(x*y+z*w),    1-2*(x*x+z*z),  2*(y*z-x*w)],
        [2*(x*z-y*w),    2*(y*z+x*w),    1-2*(x*x+y*y)],
    ])


def R_to_quat(R):
    tr = R[0,0]+R[1,1]+R[2,2]
    if tr > 0:
        s = 2*np.sqrt(tr+1.0)
        w = 0.25*s
        x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2*np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])
        w = (R[2,1]-R[1,2])/s
        x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = 2*np.sqrt(1.0+R[1,1]-R[0,0]-R[2,2])
        w = (R[0,2]-R[2,0])/s
        x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = 2*np.sqrt(1.0+R[2,2]-R[0,0]-R[1,1])
        w = (R[1,0]-R[0,1])/s
        x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    return [float(x), float(y), float(z), float(w)]


def make_T(R, t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3]  = np.asarray(t).ravel()
    return T


class TFGetter(Node):
    def __init__(self):
        super().__init__('handeye_tf_getter')
        self.buf = Buffer()
        self.lst = TransformListener(self.buf, self)


def fetch_oak_internal_tf():
    """T(oak_optical → oak_root): leído en vivo del driver OAK."""
    rclpy.init()
    node = TFGetter()
    print(f'Consultando TF interno OAK: {OAK_OPTICAL_FRAME} → {OAK_ROOT_FRAME} ...')

    deadline = time.time() + 5.0
    tf = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        try:
            tf = node.buf.lookup_transform(
                OAK_OPTICAL_FRAME, OAK_ROOT_FRAME, rclpy.time.Time())
            break
        except TransformException:
            continue

    rclpy.shutdown()
    if tf is None:
        sys.exit(
            f'\nERROR: no se pudo obtener TF {OAK_OPTICAL_FRAME} → '
            f'{OAK_ROOT_FRAME} en 5s.\n'
            f'  - Asegúrate de que el driver OAK-D esté corriendo.\n'
            f'  - Si tienes un static_transform_publisher nuestro activo '
            f'(launch previo o comando manual), párralo (Ctrl+C) antes.')

    t = tf.transform.translation
    r = tf.transform.rotation
    print(f'  ✓ Obtenida: t=[{t.x:+.4f}, {t.y:+.4f}, {t.z:+.4f}]  '
          f'q=[{r.x:+.4f}, {r.y:+.4f}, {r.z:+.4f}, {r.w:+.4f}]')
    return make_T(quat_to_R([r.x, r.y, r.z, r.w]),
                  np.array([t.x, t.y, t.z]))


def main():
    if not IN_PATH.exists():
        sys.exit(f'No existe {IN_PATH}. Ejecuta run_handeye.py antes.')

    data = json.load(open(IN_PATH))
    m = data['methods'][METHOD]

    # T(ur_I_base → oak_optical), de la calibración
    T_opt_in_ur = make_T(
        np.array(m['R_cam2base']),
        np.array(m['t_cam2base']))

    # T(oak_optical → oak_root), del driver OAK
    T_root_in_opt = fetch_oak_internal_tf()

    # Composición: ur_I_base → oak_root
    T_root_in_ur = T_opt_in_ur @ T_root_in_opt

    t_final = T_root_in_ur[:3,3]
    q_final = R_to_quat(T_root_in_ur[:3,:3])

    parent = data['base_frame']     # ur_dual_I_base_link
    child  = OAK_ROOT_FRAME         # oak-d-base-frame

    yaml_txt = f"""# Calibracion hand-eye UR5 (brazo I, MoveIt group Right_arm) <-> OAK-D
# Generado por export_handeye_yaml.py
# Metodo calibrateHandEye: {METHOD}
# Muestras: {data['n_samples']}
# Max diferencia traslacional entre metodos OpenCV: {data['max_method_diff_mm']:.2f} mm
#
# El TF se publica de ur_dual_I_base_link a oak-d-base-frame (raiz del
# arbol OAK) en lugar de oak_rgb_camera_optical_frame, para evitar el
# conflicto de doble padre. La matriz aqui es la calibrada compuesta con
# la transformacion interna OAK (optical -> root), consultada en vivo
# del driver.

handeye:
  parent_frame: "{parent}"
  child_frame:  "{child}"
  translation:
    x: {float(t_final[0]):.10f}
    y: {float(t_final[1]):.10f}
    z: {float(t_final[2]):.10f}
  rotation_quaternion:
    x: {q_final[0]:.10f}
    y: {q_final[1]:.10f}
    z: {q_final[2]:.10f}
    w: {q_final[3]:.10f}
"""
    OUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    OUT_PATH.write_text(yaml_txt)
    print(f'\n✓ Escrito {OUT_PATH}\n')
    print(yaml_txt)


if __name__ == '__main__':
    main()
