# -*- coding: utf-8 -*-
import numpy as np

def compute_overshoot_pct(q_peak, q_ss, delta):
    """
    OS% = 100*|q_peak - q_ss| / |Δ|
    delta = amplitud del paso (misma unidad que q).
    """
    if abs(delta) < 1e-12:
        return 0.0
    os_ = abs(q_peak - q_ss) / abs(delta)
    return max(0.0, 100.0 * os_)

def steady_state_mean(q_tail):
    """
    Media de las últimas muestras como aproximación de régimen permanente.
    q_tail: np.array o list del tramo final.
    """
    q_tail = np.asarray(q_tail, dtype=float)
    if q_tail.size == 0:
        return float("nan")
    return float(np.mean(q_tail))

def compute_settling_time(t, q, q_ss, band=0.02):
    """
    Tiempo de establecimiento: primer instante t en que |q - q_ss| <= band*Δ
    NOTA: aquí no conocemos Δ de forma explícita; suele evaluarse en el segmento.
    Para un primer vistazo calculamos con respecto al rango del segmento.
    """
    t = np.asarray(t, dtype=float)
    q = np.asarray(q, dtype=float)
    if t.size == 0 or q.size == 0:
        return float("nan")
    # Band respecto al rango aproximado del segmento:
    delta = max(1e-9, np.max(q) - np.min(q))
    tol = band * delta
    inside = np.abs(q - q_ss) <= tol
    for i in range(len(inside)):
        # condición “se mantiene” podría reforzarse con una ventana, aquí lo dejamos simple
        if inside[i]:
            return float(t[i] - t[0])
    return float("nan")
