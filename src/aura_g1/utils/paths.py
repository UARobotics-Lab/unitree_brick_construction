"""Utility helpers for locating project data directories and storing outputs."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np

# Project directory layout ----------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[2]
REPO_ROOT = SRC_DIR.parent
DATA_DIR = REPO_ROOT / "data"
RAW_DIR = DATA_DIR / "raw"
PROCESSED_DIR = DATA_DIR / "processed"
RESULTS_DIR = DATA_DIR / "results"
PLOTS_DIR = DATA_DIR / "plots"


def ensure_data_dirs() -> Mapping[str, Path]:
    """Create the expected data directories if they do not already exist."""

    for directory in (RAW_DIR, PROCESSED_DIR, RESULTS_DIR, PLOTS_DIR):
        directory.mkdir(parents=True, exist_ok=True)
    return {
        "raw": RAW_DIR,
        "processed": PROCESSED_DIR,
        "results": RESULTS_DIR,
        "plots": PLOTS_DIR,
    }


def _coerce_name(name: str | Path) -> Path:
    candidate = Path(name)
    # Normalise redundant separators while preserving relative subfolders
    return candidate


def path_results(name: str | Path) -> Path:
    """Return the absolute path for an artifact stored under data/results."""

    ensure_data_dirs()
    return (RESULTS_DIR / _coerce_name(name)).resolve()


def path_plots(name: str | Path) -> Path:
    """Return the absolute path for an artifact stored under data/plots."""

    ensure_data_dirs()
    return (PLOTS_DIR / _coerce_name(name)).resolve()


def save_numpy(name: str | Path, arr: np.ndarray) -> Path:
    """Persist a NumPy array in data/results and return the saved path."""

    target = path_results(name)
    target.parent.mkdir(parents=True, exist_ok=True)
    np.save(target, arr)
    return target


def save_plot(fig_or_plt: Any, name: str | Path, dpi: int = 150, **savefig_kwargs: Any) -> Path:
    """Save a Matplotlib figure/module output into data/plots."""

    target = path_plots(name)
    target.parent.mkdir(parents=True, exist_ok=True)

    if hasattr(fig_or_plt, "savefig"):
        savefig = getattr(fig_or_plt, "savefig")
    elif hasattr(fig_or_plt, "figure"):
        savefig = getattr(fig_or_plt.figure, "savefig")
    else:
        raise TypeError("Object does not expose a savefig method")

    save_kwargs = {"dpi": dpi, **savefig_kwargs}
    savefig(target, **save_kwargs)
    return target


__all__ = [
    "DATA_DIR",
    "RAW_DIR",
    "PROCESSED_DIR",
    "RESULTS_DIR",
    "PLOTS_DIR",
    "ensure_data_dirs",
    "path_results",
    "path_plots",
    "save_numpy",
    "save_plot",
]
