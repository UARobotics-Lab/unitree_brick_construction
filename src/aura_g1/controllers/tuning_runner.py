# -*- coding: utf-8 -*-
"""
Harness de tuning Kp/Kd que reusa tu controlador seguro (rutina_ladrillos.py).
Modos: routine / step / hold. Registra CSV y calcula métricas básicas.
"""
import os, sys, time, json, math, argparse
from datetime import datetime

# --- Importa tu controlador seguro (perfil quintico, DDS, etc.) ---
from src.aura_g1.controllers.rutina_ladrillos import ArmSequence  # ajusta si cambia el path

# --- Utils (logging y métricas) ---
from src.aura_g1.utils.logger_utils import LoggerCSV
from src.aura_g1.utils.metrics_utils import (
    compute_overshoot_pct, compute_settling_time, steady_state_mean
)

# ------------------ Helpers de paths ------------------
def ensure_dir(p):
    os.makedirs(p, exist_ok=True)
    return p

def default_outdir():
    ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    return ensure_dir(os.path.join("data", "results", "tuning", ts))

# ------------------ CLI ------------------
def build_parser():
    p = argparse.ArgumentParser(description="Runner de pruebas PD (Kp/Kd) con rutina segura.")
    p.add_argument("--mode", choices=["routine", "step", "hold"], default="routine")
    p.add_argument("--dds", type=str, default="loong", help="perfil o dominio DDS/SDK si aplica")
    p.add_argument("--kp", type=float, default=None)
    p.add_argument("--kd", type=float, default=None)

    # Routine mode
    p.add_argument("--routine_json", type=str, default=None, help="Ruta a rutina .json (si omites, usa tu por defecto)")

    # Step mode
    p.add_argument("--joint", type=int, default=None, help="Índice absoluto de la junta (según SDK)")
    p.add_argument("--amp_deg", type=float, default=20.0)
    p.add_argument("--duration_s", type=float, default=0.8, help="tiempo de cada movimiento")
    p.add_argument("--reps", type=int, default=6)

    # Hold mode
    p.add_argument("--hold_deg", type=float, nargs="+", default=[0.0, 30.0, 60.0])
    p.add_argument("--hold_time_s", type=float, default=3.0)

    # Logging/seguridad
    p.add_argument("--outdir", type=str, default=None)
    p.add_argument("--log_rate_hz", type=float, default=500.0)
    p.add_argument("--max_vel_dps", type=float, default=120.0)     # si lo usa tu seq
    p.add_argument("--max_current_A", type=float, default=0.6)     # si lo expone tu SDK
    p.add_argument("--soft_limits", type=str, default=None, help="JSON con límites suaves por junta")
    return p

# ------------------ Lecturas seguras del estado ------------------
def read_joint_state(seq, j_idx):
    """ Devuelve (q, qd, i_or_tau) de la junta j_idx, tolerante a SDK. """
    q = 0.0; qd = 0.0; i_or_tau = 0.0
    ls = seq.low_state
    try:
        st = ls.motor_state[j_idx]
        # Ajusta nombres según tu SDK:
        q = getattr(st, "q", q)
        qd = getattr(st, "dq", qd)
        for cand in ("tauEst", "tau", "i", "iq"):
            if hasattr(st, cand):
                i_or_tau = getattr(st, cand)
                break
    except Exception:
        pass
    return q, qd, i_or_tau

# ------------------ Sampler de logging ------------------
def sample_loop(seq, logger: LoggerCSV, joints, rate_hz, t_stop):
    """Muestrea a rate_hz hasta t_stop() sea True. """
    dt = 1.0 / max(1.0, rate_hz)
    while not t_stop():
        t_s = time.time()
        for j in joints:
            q, qd, i_or_tau = read_joint_state(seq, j)
            # qref: tomamos del objetivo actual si lo tienes accesible; de lo contrario, guarda NaN
            qref = math.nan
            logger.append(t_s, j, q, qref, qd, i_or_tau)
        # sleep ajustado
        t_spent = time.time() - t_s
        if dt - t_spent > 0:
            time.sleep(dt - t_spent)

# ------------------ Pruebas ------------------
def run_routine(seq: ArmSequence, args, outdir):
    # Arranque e inicialización
    seq.Init()
    seq.Start()

    # Ganancias
    if args.kp is not None and args.kd is not None:
        seq.set_gains(args.kp, args.kd)  # debes tener este setter sencillo en tu ArmSequence

    # Carga de rutina JSON
    if args.routine_json is None:
        print("[rutina] No se indicó --routine_json; usaré la rutina por defecto de tu script.")
    else:
        with open(args.routine_json, "r", encoding="utf-8") as f:
            routine_data = json.load(f)
        seq.load_routine(routine_data)  # implementa un wrapper si tu secuencia ya lo hace internamente

    # Logging
    joints = seq.arm_joints  # ajusta según tu clase; si no existe, pásalo por args
    logger = LoggerCSV(os.path.join(outdir, f"routine_Kp{seq.kp}_Kd{seq.kd}.csv"))
    stop_flag = {"stop": False}
    t_start = time.time()

    def should_stop():
        return stop_flag["stop"]

    # Inicia muestreo en background (simple: hilo pobre-man)
    import threading
    t = threading.Thread(target=sample_loop, args=(seq, logger, joints, args.log_rate_hz, should_stop))
    t.daemon = True
    t.start()

    # Ejecuta rutina
    seq.run()  # O llama a tu método que recorre las poses del JSON con move_to

    # Cierra logging
    stop_flag["stop"] = True
    t.join(timeout=2.0)
    logger.save()

    print(f"[rutina] Log guardado en: {logger.path}")
    return logger.path

def run_step(seq: ArmSequence, args, outdir):
    assert args.joint is not None, "Debes indicar --joint para modo step."
    seq.Init()
    seq.Start()
    if args.kp is not None and args.kd is not None:
        seq.set_gains(args.kp, args.kd)

    j = args.joint
    amp = math.radians(args.amp_deg)  # si tu SDK usa rad; si usa grados, elimina radians

    # Lee posición base
    while seq.low_state is None:
        time.sleep(0.02)
    q0, _, _ = read_joint_state(seq, j)

    logger = LoggerCSV(os.path.join(outdir, f"step_joint{j}_Kp{seq.kp}_Kd{seq.kd}.csv"))
    stop_flag = {"stop": False}

    import threading
    t = threading.Thread(target=sample_loop, args=(seq, logger, [j], args.log_rate_hz, lambda: stop_flag["stop"]))
    t.daemon = True
    t.start()

    # Reps: centro -> up -> centro -> down -> centro ...
    for _ in range(args.reps):
        seq.move_to({j: q0 + amp}, duration=args.duration_s)
        seq.move_to({j: q0},          duration=args.duration_s)
        seq.move_to({j: q0 - amp},    duration=args.duration_s)
        seq.move_to({j: q0},          duration=args.duration_s)

    stop_flag["stop"] = True
    t.join(timeout=2.0)
    logger.save()

    # Métricas básicas a partir del CSV (qref no está; estimamos Δ con q final-inicial o usa tu referencia)
    try:
        import pandas as pd
        df = pd.read_csv(logger.path)
        # Segmenta una subida (toma 1er ciclo: centro->up)
        # Para simplificar: usa rango temporal de la primera subida
        t0 = df["t_s"].min()
        t_end = t0 + (args.duration_s * 1.2)
        seg = df[(df["t_s"] >= t0) & (df["t_s"] <= t_end)]
        q_ss = steady_state_mean(seg["q"].values[-int(0.2*len(seg)) :])  # último 20% como "ss"
        q_peak = seg["q"].max()
        d = max(1e-6, abs((q0 + amp) - q0))
        os_pct = compute_overshoot_pct(q_peak, q_ss, d)
        t_settle = compute_settling_time(seg["t_s"].values, seg["q"].values, q_ss, band=0.02)
        summary = {
            "mode": "step", "joint": j, "Kp": seq.kp, "Kd": seq.kd,
            "overshoot_pct": os_pct, "settling_time_s": t_settle
        }
        with open(os.path.join(outdir, f"step_joint{j}_Kp{seq.kp}_Kd{seq.kd}_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        print("[step] Summary:", summary)
    except Exception as e:
        print("[step] No se pudo calcular summary (ok para primera versión):", e)

    return logger.path

def run_hold(seq: ArmSequence, args, outdir):
    assert args.joint is not None, "Debes indicar --joint para modo hold."
    seq.Init()
    seq.Start()
    if args.kp is not None and args.kd is not None:
        seq.set_gains(args.kp, args.kd)

    j = args.joint
    while seq.low_state is None:
        time.sleep(0.02)
    qbase, _, _ = read_joint_state(seq, j)

    logger = LoggerCSV(os.path.join(outdir, f"hold_joint{j}_Kp{seq.kp}_Kd{seq.kd}.csv"))
    stop_flag = {"stop": False}
    import threading
    t = threading.Thread(target=sample_loop, args=(seq, logger, [j], args.log_rate_hz, lambda: stop_flag["stop"]))
    t.daemon = True
    t.start()

    for deg in args.hold_deg:
        qh = qbase + math.radians(deg)
        seq.move_to({j: qh}, duration=0.8)
        t0 = time.time()
        while (time.time() - t0) < args.hold_time_s:
            time.sleep(0.02)

    # volver al base
    seq.move_to({j: qbase}, duration=0.8)

    stop_flag["stop"] = True
    t.join(timeout=2.0)
    logger.save()
    print(f"[hold] Log guardado en: {logger.path}")
    return logger.path

# ------------------ main ------------------
def main():
    args = build_parser().parse_args()
    outdir = default_outdir() if args.outdir is None else ensure_dir(args.outdir)

    # Inicializa tu secuencia (DDS/domínio, etc.). Si tu ArmSequence requiere args distintos, ajusta aquí.
    seq = ArmSequence()

    # Llama el modo
    if args.mode == "routine":
        run_routine(seq, args, outdir)
    elif args.mode == "step":
        run_step(seq, args, outdir)
    elif args.mode == "hold":
        run_hold(seq, args, outdir)
    else:
        raise ValueError("Modo no soportado.")

if __name__ == "__main__":
    main()
