# -*- coding: utf-8 -*-
import os, csv

class LoggerCSV:
    """
    Logger simple para guardar muestras por junta.
    Columnas: t_s, joint, q, qref, dq, i_or_tau
    """
    def __init__(self, path_csv: str):
        self.path = path_csv
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        self._rows = []
        # encabezado
        self._rows.append(["t_s","joint","q","qref","dq","i_or_tau"])

    def append(self, t_s, joint, q, qref, dq, i_or_tau):
        self._rows.append([float(t_s), int(joint), float(q), float(qref) if qref==qref else "", float(dq), float(i_or_tau)])

    def save(self):
        with open(self.path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerows(self._rows)
