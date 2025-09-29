"""Thin compatibility layer for centralized data IO helpers."""
from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .paths import (
    ensure_data_dirs as _ensure_data_dirs,
    path_plots as _path_plots,
    path_results as _path_results,
    save_numpy as _save_numpy,
    save_plot as _save_plot,
)


def ensure_data_dirs() -> Mapping[str, Any]:
    """Create default data directories and return their mapping."""

    return _ensure_data_dirs()


def path_results(name: str) -> Any:
    """Return the absolute path inside data/results for name."""

    return _path_results(name)


def path_plots(name: str) -> Any:
    """Return the absolute path inside data/plots for name."""

    return _path_plots(name)


def save_numpy(name: str, arr: np.ndarray) -> Any:
    """Store arr under data/results and return the resulting path."""

    return _save_numpy(name, arr)


def save_plot(fig_or_plt: Any, name: str, dpi: int = 150, **kwargs: Any) -> Any:
    """Persist a Matplotlib figure in data/plots and return its path."""

    return _save_plot(fig_or_plt, name, dpi=dpi, **kwargs)


__all__ = [
    "ensure_data_dirs",
    "path_results",
    "path_plots",
    "save_numpy",
    "save_plot",
]
