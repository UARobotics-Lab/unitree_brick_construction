"""Rutina de prueba para generar poses a partir de pallets y resolver IK."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table
from spatialmath import SE3, SO3
from spatialmath.base import trprint  # noqa: F401

# Asegurar que el paquete aura_g1 esté disponible al ejecutar como script
REPO_ROOT = Path(__file__).resolve().parent.parent
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in map(str, sys.path):
    sys.path.insert(0, str(SRC_DIR))

from aura_g1.kinematics.ik.IK import LM_Chan, NR  # noqa: E402
from aura_g1.kinematics.ik.pallet import Brick, Pallet  # noqa: E402
from aura_g1.tasks.mover_ladrillo import MoverLadrillo  # noqa: E402
from aura_g1.utils.io_paths import ensure_data_dirs, path_results, save_numpy  # noqa: E402

import roboticstoolbox as rtb  # noqa: E402
from roboticstoolbox import Robot  # noqa: E402
from roboticstoolbox.robot.ETS import ETS  # noqa: E402

# ---------------------------------------------------------------------------
# Configuración general y utilidades
# ---------------------------------------------------------------------------

ensure_data_dirs()

# ---------------------------------------------------------------------------
# Carga del robot Aura y configuración del solver
# ---------------------------------------------------------------------------

URDF_PATH = SRC_DIR / "aura_g1" / "kinematics" / "ik" / "g1_dual_arm_left.urdf"
robot = Robot.URDF(str(URDF_PATH))
robot.qr = [0.0] * robot.n
print(" Robot Aura cargado.")
print(robot.fkine(robot.qr))


def robust_solve(self, Tep, q0=None):
    """Monkey patch de ETS.solve para garantizar índices enteros."""

    max_jindex: int = int(np.max(self.jindices))
    q0_method = np.zeros((self.slimit, max_jindex + 1))

    jindices = np.asarray(self.jindices).astype(int)
    print("DEBUG >>> jindices:", jindices, type(jindices))

    if q0 is None:
        q0_method[:, jindices] = self._random_q(self, self.slimit)
    elif not isinstance(q0, np.ndarray):
        q0 = np.array(q0)

    if q0 is not None and q0.ndim == 1:
        q0_method[:, jindices] = self._random_q(self, self.slimit)
        q0_method[0, jindices] = q0

    if q0 is not None and q0.ndim == 2:
        q0_method[:, jindices] = self._random_q(self, self.slimit)
        q0_method[:q0.shape[0], self.jindices] = q0

    return q0_method


ETS.solve = robust_solve
ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)

# ---------------------------------------------------------------------------
# Definición de pallets y solver
# ---------------------------------------------------------------------------

brick = Brick(width=0.1, length=0.2, height=0.06)
z = 0.10
orientacion = SO3.Rz(np.deg2rad(90))
layout_mode = "inclinado"

pallet1_pose = SE3(0.1, 0.15, z) * SE3(orientacion)
pallet2_pose = SE3(0.1, 0.15, z) * SE3(orientacion)

pallet1 = Pallet(rows=1, cols=3, layers=3, brick=brick, base_pose=pallet1_pose, layout=layout_mode)
pallet2 = Pallet(rows=1, cols=3, layers=3, brick=brick, base_pose=pallet2_pose, layout=layout_mode)

solver = LM_Chan(ilimit=40, slimit=30, tol_min=1e-6, tol_max=1e-3)
np.random.seed(42)
q0 = np.tile(robot.qr, (solver.slimit, 1)) + np.random.uniform(-0.3, 0.3, (solver.slimit, robot.n))

mover = MoverLadrillo(robot=robot, solver=solver, ets=ets, pallet1=pallet1, pallet2=pallet2)
posiciones_origen = [(0, 0, 0), (0, 1, 0)]
posiciones_destino = [(0, 0, 1), (0, 1, 1)]
rutina = mover.mover(posiciones_origen, posiciones_destino, cintura_giro_rad=np.deg2rad(90))

cintura_por_pallet = {
    "Pallet 1": 0.0,
    "Pallet 2": -1.57,
}

json_path = path_results("rutina_ladrillo_prueba.json")
with open(json_path, "w", encoding="utf-8") as f:
    json.dump(rutina, f, indent=4)
print(f"Rutina de movimiento guardada en: {json_path}")

# ---------------------------------------------------------------------------
# Evaluación de IK en todos los ladrillos
# ---------------------------------------------------------------------------

console = Console()
table = Table(title="Resultados de IK", show_lines=True)

for column in ("Pallet", "Fila", "Columna", "Layer", "Cintura (rad)", "Estado", "Iteraciones", "Error", "q solución"):
    table.add_column(column, justify="left")

q_steps: list[np.ndarray] = []
pallets = {"Pallet 1": pallet1, "Pallet 2": pallet2}

for nombre, pallet in pallets.items():
    console.rule(f"[bold blue] Revisión de {nombre} [/bold blue]")
    cintura_yaw = cintura_por_pallet[nombre]

    for row in range(pallet.rows):
        for col in range(pallet.cols):
            for layer in range(pallet.layers):
                T_goal = pallet.get_pose(row, col, layer)
                q, success, its, searches, E, valid, t_total = solver.solve(ets, T_goal.A, q0=q0)

                estado = "ÉXITO" if success else "FALLO"
                iteraciones = str(its) if success else "-"
                error = f"{E:.2e}" if success else "0"
                q_str = f"[green]{np.round(q, 4).tolist()}[/green]" if success else "[red]―[/red]"
                style = "bold green" if success else "bold red"

                for paso in rutina:
                    if "brazo" in paso:
                        q_rutina = np.array(paso["brazo"])
                        if np.allclose(q[: len(q_rutina)], q_rutina, atol=1e-3):
                            cintura_yaw = paso["cintura"].get("12", 0.0)
                            break

                table.add_row(
                    nombre,
                    str(row),
                    str(col),
                    str(layer),
                    f"{cintura_yaw:.2f}",
                    estado,
                    iteraciones,
                    error,
                    q_str,
                    style=style,
                )

                console.print(f"\n[bold]Pose objetivo (SE3) para ({row}, {col}, {layer}):[/bold]")
                console.print(np.array_str(T_goal.A, precision=4, suppress_small=True))

                if success:
                    q_rounded = np.round(q, 4).tolist()
                    console.print(f"[green]q solución para ladrillo ({row}, {col}):[/green] {q_rounded}")
                    q_steps.append(q.copy())
                else:
                    console.print("[red]↳ No se pudo alcanzar la pose objetivo.[/red]")

                console.rule("")

console.print(table)

q_steps_arr = np.array(q_steps)
npy_path = save_numpy("q_steps_pallet_LM_left.npy", q_steps_arr)
csv_path = path_results("q_steps_pallet_LM_left.csv")
np.savetxt(csv_path, q_steps_arr, delimiter=",")
print(f"Pasos de IK guardados en: {npy_path}")
print(f"Pasos de IK (CSV) guardados en: {csv_path}")
