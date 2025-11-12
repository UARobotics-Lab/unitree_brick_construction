# -*- coding: utf-8 -*-
"""
Wrapper para lanzar el tuning_runner con un archivo de configuración JSON opcional.
"""
import os, sys, json, argparse, subprocess

def build_parser():
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=str, default=None, help="Archivo JSON con lotes de pruebas")
    # Passthrough: permite usar los mismos args del runner sin config
    p.add_argument("--mode", type=str, help="routine/step/hold")
    p.add_argument("--dds", type=str)
    p.add_argument("--kp", type=float)
    p.add_argument("--kd", type=float)
    p.add_argument("--routine_json", type=str)
    p.add_argument("--joint", type=int)
    p.add_argument("--amp_deg", type=float)
    p.add_argument("--duration_s", type=float)
    p.add_argument("--reps", type=int)
    p.add_argument("--hold_deg", type=float, nargs="+")
    p.add_argument("--hold_time_s", type=float)
    p.add_argument("--outdir", type=str)
    p.add_argument("--log_rate_hz", type=float)
    return p

def run_one(args_dict):
    # Construye el comando hacia tuning_runner.py
    cmd = [sys.executable, "src/aura_g1/controllers/tuning_runner.py"]
    for k, v in args_dict.items():
        if v is None: 
            continue
        if isinstance(v, bool):
            if v: cmd.append(f"--{k}")
        elif isinstance(v, (list, tuple)):
            cmd.append(f"--{k}")
            cmd += [str(x) for x in v]
        else:
            cmd += [f"--{k}", str(v)]
    print("[run]", " ".join(cmd))
    subprocess.run(cmd, check=True)

def main():
    args = build_parser().parse_args()
    if args.config is None:
        # modo passthrough (una sola corrida)
        d = {k:v for k,v in vars(args).items() if k != "config" and v is not None}
        run_one(d)
    else:
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        base = cfg.get("base_args", {})
        runs = cfg.get("runs", [])
        for r in runs:
            d = dict(base); d.update(r)
            run_one(d)

if __name__ == "__main__":
    main()
