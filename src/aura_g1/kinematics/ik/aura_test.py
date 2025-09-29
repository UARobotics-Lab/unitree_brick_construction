"""
aura_test.py

Este script:
- Carga Aura (brazo) desde URDF.
- Verifica la cadena ETS generada.
- Prueba la cinemática directa (FK).
- Prueba la IK de la Toolbox interna.
- Prueba solver NR personalizado (desde aura_ik_methods.py).
- Compara los métodos NR, GN y LM (Chan), y además ejecuta un barrido multi-escenario.

Flujo general:
1. Ajusta el solver ETS para que utilice índices de articulaciones enteros.
2. Carga el URDF del brazo Aura y obtiene la cadena ETS.
3. Evalúa la FK para una configuración nominal y resuelve IK con múltiples métodos.
4. Genera reportes (plots/tablas/CSV) y ejecuta escenarios adicionales con variación en la meta.
5. Consulta la utilería de pallets para evaluar poses de ladrillos.

Es necesario:
- Tener IK.py en la misma carpeta.
- Haber activado entorno conda: conda activate dktutorial
"""
from pallet import Pallet

# Permite ejecutar el script directamente (python path a src/)
if __package__ in (None, ""):
    import sys
    from pathlib import Path

    SRC_PATH = Path(__file__).resolve().parents[3]
    if str(SRC_PATH) not in map(str, sys.path):
        sys.path.insert(0, str(SRC_PATH))

from aura_g1.utils.paths import ensure_data_dirs, path_results, save_numpy, save_plot
# --- Librerías base ---
import roboticstoolbox as rtb 
import pandas as pd

from spatialmath import SE3, SO3
import numpy as np
from roboticstoolbox.robot.ETS import ETS

# --- Importar la clase ---
from IK import NR, LM_Chan, GN, LM_Sugihara  # metodos de IK personalizados

ensure_data_dirs()

# Monkey patch para reescribir el método eval de ETS
def robust_solve(self, Tep, q0=None):
    """Versiona ETS.solve para garantizar índices enteros y seeds reproducibles."""

    # Copia la lógica base del toolbox, ajustándola al robot Aura
    max_jindex: int = int(np.max(self.jindices))  # Identificador más alto de articulación

    # Reserva la matriz con slots para los intentos de búsqueda (slimit)
    q0_method = np.zeros((self.slimit, max_jindex + 1))

    # Asegura que los índices de articulación sean enteros antes de indexar
    jindices = np.asarray(self.jindices).astype(int)
    print("DEBUG >>> jindices:", jindices, type(jindices))

    if q0 is None:
        # Genera seeds aleatorios cuando no se provee q0
        q0_method[:, jindices] = self._random_q(self, self.slimit)

    elif not isinstance(q0, np.ndarray):
        # Normaliza listas/tuplas a ndarray
        q0 = np.array(q0)

    if q0 is not None and q0.ndim == 1:
        # Usa q0 como primera fila y llena el resto con seeds aleatorios
        q0_method[:, jindices] = self._random_q(self, self.slimit)
        q0_method[0, jindices] = q0

    if q0 is not None and q0.ndim == 2:
        # Copia el lote suministrado y complementa con seeds aleatorios
        q0_method[:, jindices] = self._random_q(self, self.slimit)
        q0_method[:q0.shape[0], self.jindices] = q0

    return q0_method

ETS.solve = robust_solve

print(" Método eval de ETS modificado para usar jindices como enteros.")
# ETS: Elementary Transformation Sequence-> Cadena cinematica usando transformaciones elementales

from roboticstoolbox import Robot
from spatialmath import SE3
# ---  Carga el URDF ---
import os
#from roboticstoolbox import Robot

#  Ruta al URDF del robot Aura
urdf_path = os.path.abspath("g1_dual_arm_left.urdf") #URDF modificado para Aura
print(" Usando URDF:", urdf_path)

robot = Robot.URDF(urdf_path)

robot.qr = [0.0]*robot.n

print("Configuracion  (qr) definida:", robot.qr)


print(" Robot Aura cargado.")
print(f"Grados de libertad: {robot.n}") #En este caso, 7 DOF del brazo de Aura

print(f"link base del robot :{robot.base_link}")  # Muestra el nombre del link raíz

print(robot.fkine(robot.qr))  # FK en la pose neutral


# ---  Ver ETS ---
ets = robot.ets(end="left_rubber_hand")
ets.jindices = np.array(ets.jindices, dtype=int)#Forzamos jindices a tipo entero para evitar problemas con el solver
print(f"ETS: {ets}")
print(" jindices: {ets.jindices}")

# --- FK simple con configuración por defecto ---
Te_fk = ets.eval(robot.qr)
print(" FK de configuración por defecto (qr):")
print(Te_fk)

# --- Definir rotación ---

theta_x = np.deg2rad(90) # Rotación de 90 grados en radianes
R = SO3.Rx(theta_x)  # Rotación alrededor del eje X

# --- Definir posicion ---

x, y, z = 0.3, 0.3, 0.0  # Posición deseada

#T_goal = SE3(x, y, z) * SE3(R)  # Pose deseada con rotación y posición
#print(f"Pose objetivo:\n{T_goal}")



# --- IK usando toolbox integrada ---
T_goal = SE3(0.3, 0.3, 0.2)  #  Pose deseada a la que llegara en este caso left_rubber_hand
print(f"Pose objetivo:\n{T_goal}")

# ---  Solver de IK usando NR ---
# Genera inicio de búsqueda
np.random.seed(42)  # Para reproducibilidad global del script
# Crea un punto de partida aleatorio para los solvers
solver_NR = NR(pinv=True, ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3)
q0 = np.random.uniform(-1, 1, (solver_NR.slimit, robot.n))  # 30 intentos de inicio aleatorio

#q0 = np.tile(robot.qr, (solver.slimit, 1))  

print(f"\n Probando solver NR: {solver_NR.name}")

# Resolver con Newton-Raphson
q_NR, success_NR, its_NR, searches_NR, E_NR, jl_valid_NR, t_NR = solver_NR.solve(ets, T_goal.A, q0)
print(" IK con NR:")
print(f"q: {q_NR}")
print(f"FK del resultado NR: {ets.eval(q_NR)}")
print(f"Converge: {success_NR} | Iteraciones: {its_NR} | Busquedas: {searches_NR}")
print(f"Error E: {E_NR} | Dentro de límites: {jl_valid_NR} | Tiempo total: {t_NR:.4f}s")


# ---  Solver de IK usando GN ---
solver_GN = GN(pinv=True, ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3)
print(f"\n Probando solver GN: {solver_GN.name}")
q_GN, success_GN, its_GN, searches_GN, E_GN, jl_valid_GN, t_GN = solver_GN.solve(ets, T_goal.A, q0)
print(" IK con GN:")
print(f"q: {q_GN}")
print(f"FK del resultado GN: {ets.eval(q_GN)}")
print(f"Converge: {success_GN} | Iteraciones: {its_GN} | Busquedas: {searches_GN}")
print(f"Error E: {E_GN} | Dentro de límites: {jl_valid_GN} | Tiempo total: {t_GN:.4f}s")


# ---  Solver de IK usando LM (Chan) ---
solver_LM = LM_Chan(ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3)
print(f"\n Probando solver LM: {solver_LM.name}")
# Resolver con LM (Chan)
q_LM, success_LM, its_LM, searches_LM, E_LM, jl_valid_LM, t_LM = solver_LM.solve(ets, T_goal.A, q0)

# --- Guardar q_LM ---
resultado_path = save_numpy("q_LM_result.npy", q_LM)
print(f"q_LM guardado en: {resultado_path}")

print(" IK con LM:")
print(f"q: {q_LM}")
print(f"FK del resultado LM: {ets.eval(q_LM)}")
print(f"Converge: {success_LM} | Iteraciones: {its_LM} | Busquedas: {searches_LM}")
print(f"Error E: {E_LM} | Dentro de límites: {jl_valid_LM} | Tiempo total: {t_LM:.4f}s")


#Pasos de q para llegar a la solución
# --- Comparación de métodos: gráficos y tabla ---
try:
    import matplotlib.pyplot as plt

    metodos = ["NR", "GN", "LM (Chan)"]
    tiempos = [t_NR, t_GN, t_LM]
    iteraciones = [its_NR, its_GN, its_LM]
    errores = [E_NR, E_GN, E_LM]

    fig, axes = plt.subplots(1, 3, figsize=(12, 4))

    # Tiempo
    axes[0].bar(metodos, tiempos, color=["#1f77b4", "#ff7f0e", "#2ca02c"]) 
    axes[0].set_title("Tiempo (s)")
    for i, v in enumerate(tiempos):
        axes[0].text(i, v, f"{v:.3f}", ha="center", va="bottom", fontsize=8)

    # Iteraciones
    axes[1].bar(metodos, iteraciones, color=["#1f77b4", "#ff7f0e", "#2ca02c"]) 
    axes[1].set_title("Iteraciones")
    for i, v in enumerate(iteraciones):
        axes[1].text(i, v, f"{int(v) if not np.isnan(v) else 'NaN'}", ha="center", va="bottom", fontsize=8)

    # Error final E
    axes[2].bar(metodos, errores, color=["#1f77b4", "#ff7f0e", "#2ca02c"]) 
    axes[2].set_title("Error (E)")
    for i, v in enumerate(errores):
        axes[2].text(i, v, f"{v:.2e}", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    plot_path = save_plot(plt, "ik_methods_comparison.png", dpi=150)
    print(f"Gráfica comparativa guardada en: {plot_path}")
except Exception as ex:
    print(f"No se pudo generar la gráfica comparativa: {ex}")

# Tabla comparativa
def fmt_bool(b):
    """Convierte valores booleanos/NaN en etiquetas amigables."""

    try:
        return "Sí" if bool(b) else "No"
    except:
        return "-"

rows = [
    ("NR", fmt_bool(success_NR), int(0 if np.isnan(searches_NR) else searches_NR), int(0 if np.isnan(its_NR) else its_NR), f"{t_NR:.4f}", f"{E_NR:.3e}"),
    ("GN", fmt_bool(success_GN), int(0 if np.isnan(searches_GN) else searches_GN), int(0 if np.isnan(its_GN) else its_GN), f"{t_GN:.4f}", f"{E_GN:.3e}"),
    ("LM (Chan)", fmt_bool(success_LM), int(0 if np.isnan(searches_LM) else searches_LM), int(0 if np.isnan(its_LM) else its_LM), f"{t_LM:.4f}", f"{E_LM:.3e}"),
]

header = ("Método", "Converge", "Búsquedas", "Iteraciones", "Tiempo(s)", "Error(E)")
print("\nTabla comparativa (métodos vs. tiempo/iteraciones/error):")
print(" | ".join(header))
print("-" * 75)
for r in rows:
    print(" | ".join(str(x) for x in r))


# =========================
# Pruebas multi-escenario
# =========================
print("\n=== Pruebas multi-escenario (NR, GN, LM-Chan, LM-Sugihara) ===")

# Conjunto de posiciones objetivo (mantener en zona alcanzable)
targets = [
    SE3(0.30, 0.30, 0.20),  # similar a la prueba base
    SE3(0.35, 0.25, 0.25),
    SE3(0.28, 0.35, 0.18),
    SE3(0.25, 0.22, 0.15),
    SE3(0.32, 0.28, 0.30),
]

# Configurar solvers a comparar
def make_solvers():
    """Crea nuevas instancias de los solvers para cada escenario."""

    return {
        "NR": NR(pinv=True, ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3),
        "GN": GN(pinv=True, ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3),
        "LM (Chan)": LM_Chan(ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3),
        "LM (Sugihara)": LM_Sugihara(λ=1.0, ilimit=30, slimit=30, tol_min=1e-6, tol_max=1e-3),
    }

results = []  # Acumula métricas de todos los escenarios

for idx, Tg in enumerate(targets):
    print(f"\n-- Escenario {idx+1}/{len(targets)}: T_goal = {Tg}")
    solvers = make_solvers()
    # Semilla reproducible por escenario para q0 (mismo q0 entre métodos en un escenario)
    rng = np.random.default_rng(1000 + idx)  # Cambiar la seed modifica la nube de q0 por escenario
    slimit = list(solvers.values())[0].slimit
    q0_batch = rng.uniform(-1, 1, (slimit, robot.n))

    for name, solver in solvers.items():
        q, success, its, searches, E, jl_valid, t = solver.solve(ets, Tg.A, q0_batch)
        Te = ets.eval(q)
        # Convertir a matrices numpy independientemente de si es SE3 o ndarray
        TgA = Tg.A if hasattr(Tg, 'A') else np.asarray(Tg)
        TeA = Te.A if hasattr(Te, 'A') else np.asarray(Te)
        # Errores de precisión
        p_goal = TgA[:3, 3]
        p_est = TeA[:3, 3]
        pos_err = float(np.linalg.norm(p_goal - p_est))
        R_goal = TgA[:3, :3]
        R_est = TeA[:3, :3]
        R_err = R_goal @ R_est.T
        tr = np.trace(R_err)
        tr = float(np.clip((tr - 1.0) / 2.0, -1.0, 1.0))
        rot_err = float(np.arccos(tr))  # [rad]

        results.append({
            "escenario": idx + 1,
            "target_x": float(p_goal[0]),
            "target_y": float(p_goal[1]),
            "target_z": float(p_goal[2]),
            "metodo": name,
            "converge": bool(success),
            "busquedas": float(searches) if searches is not None else np.nan,
            "iteraciones": float(its) if its is not None else np.nan,
            "tiempo_s": float(t),
            "E": float(E),
            "pos_err_m": pos_err,
            "rot_err_rad": rot_err,
        })

# Exportar resultados tabulares
try:
    import pandas as pd
    df = pd.DataFrame(results)
    csv_path = path_results("ik_multi_scenario_results.csv")
    df.to_csv(csv_path, index=False)
    print(f"\nResultados exportados a CSV: {csv_path}")
    print("Resumen por método (promedio entre escenarios):")
    summary = df.groupby("metodo")[
        ["tiempo_s", "iteraciones", "busquedas", "E", "pos_err_m", "rot_err_rad"]
    ].mean(numeric_only=True)
    # Imprimir tabla compacta
    print(summary)
except Exception as ex:
    print(f"No se pudo usar pandas, imprimiendo tabla básica. Causa: {ex}")
    # Tabla simple en consola
    methods = sorted(set(r["metodo"] for r in results))
    print("\nPromedios por método (sobre escenarios):")
    header = ("Método", "Tiempo[s]", "Iterac.", "Búsq.", "E", "PosErr[m]", "RotErr[rad]")
    print(" | ".join(header))
    print("-" * 85)
    for m in methods:
        vals = [r for r in results if r["metodo"] == m]
        def mean_safe(key):
            arr = np.array([v[key] for v in vals], dtype=float)
            return float(np.nanmean(arr))
        print(" | ".join([
            m,
            f"{mean_safe('tiempo_s'):.4f}",
            f"{mean_safe('iteraciones'):.1f}",
            f"{mean_safe('busquedas'):.1f}",
            f"{mean_safe('E'):.2e}",
            f"{mean_safe('pos_err_m'):.4f}",
            f"{mean_safe('rot_err_rad'):.4f}",
        ]))

# Gráficos comparativos multi-escenario (promedio + desvío)
try:
    import matplotlib.pyplot as plt
    import math

    # Preparar datos agregados
    methods = sorted(set(r["metodo"] for r in results))
    def agg(metric):
        vals = []
        for m in methods:
            arr = np.array([r[metric] for r in results if r["metodo"] == m], dtype=float)
            vals.append((float(np.nanmean(arr)), float(np.nanstd(arr))))
        return vals

    agg_time = agg("tiempo_s")
    agg_iter = agg("iteraciones")
    agg_pos = agg("pos_err_m")
    agg_E = agg("E")

    means = lambda arr: [a for a, _ in arr]
    stds = lambda arr: [b for _, b in arr]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes.ravel()
    x = np.arange(len(methods))

    # Tiempo
    ax[0].bar(methods, means(agg_time), yerr=stds(agg_time), capsize=4)
    ax[0].set_title("Tiempo (s) [promedio ± desvío]")
    # Iteraciones
    ax[1].bar(methods, means(agg_iter), yerr=stds(agg_iter), capsize=4)
    ax[1].set_title("Iteraciones [promedio ± desvío]")
    # Error posicional
    ax[2].bar(methods, means(agg_pos), yerr=stds(agg_pos), capsize=4)
    ax[2].set_title("Error posicional (m) [promedio ± desvío]")
    # Error escalar E
    ax[3].bar(methods, means(agg_E), yerr=stds(agg_E), capsize=4)
    ax[3].set_title("Error E [promedio ± desvío]")

    for a in ax:
        a.grid(True, axis='y', linestyle='--', alpha=0.4)

    plt.tight_layout()
    plot_path2 = save_plot(plt, "ik_multi_scenario_summary.png", dpi=150)
    print(f"Gráfica resumen multi-escenario guardada en: {plot_path2}")
except Exception as ex:
    print(f"No se pudo generar la gráfica multi-escenario: {ex}")

# Toolbox solver interno (prueba base)
#q_toolbox, success, _ = robot.ikine_LM(T_goal)
#q_custom, success, _, _, _,_, _ = solver.solve(ets, T_goal.A, q0)
#print(" Mi ETS.solve:", ETS.solve)
#print(" ets.solve:", ets.solve)

""" print(" IK toolbox:")
print(f"q: {q_toolbox}")
print(f"FK del resultado toolbox: {robot.fkine(q_toolbox)}") """



#q_custom, success, its, searches, E, jl_valid, t = solver.solve(ets, T_goal.A, q0)

#print(" IK con TU NR:")
""" print(f"q: {q_custom}")
print(f"FK del resultado NR: {ets.eval(q_custom)}")
print(f"Converge: {success} | Iteraciones: {its} | Busquedas: {searches}")
print(f"Error E: {E} | Dentro de límites: {jl_valid} | Tiempo total: {t:.4f}s")
 """
print("\n Prueba finalizada.")

# Posicion del pallet (utilidad para pruebas de manipulación de ladrillos)

pallet_pose = SE3(0.4, 0, 0)  # Posición base del pallet respecto a coordenadas globales

layout_mode = "ordenado"  # Cambiar a "inclinado" para filas intercaladas
# Construye un pallet simple para recuperar poses de ladrillos de prueba
pallet = Pallet(
    rows=3,
    cols=4,
    layers=1,
    brick=(0.1, 0.2, 0.06),
    base_pose=pallet_pose,
    layout=layout_mode,
)

print("Pallet creado con las siguientes poses:")

pose_1_2 = pallet.get_pose(1, 2, 0)
print(f"Pose del ladrillo en fila 1, columna 2, capa 0: {pose_1_2}")
