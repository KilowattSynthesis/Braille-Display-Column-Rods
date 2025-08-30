"""Retainer to go around a sewing needle/"dot"."""

import copy
import json
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import product
from pathlib import Path

import build123d as bd
import build123d_ease as bde
import git
from build123d_ease import show
from loguru import logger


@dataclass(kw_only=True)
class MainSpec:
    """Specification."""

    main_od: float = 4.0
    main_id: float = 0.8
    main_length: float = 3.0

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {}
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)


def dot_retainer(spec: MainSpec) -> bd.Part | bd.Compound:
    """Make the dot_retainer."""
    p = bd.Part(None)

    # Draw the dot_retainer.
    p += bd.Cylinder(
        spec.main_od / 2,
        height=spec.main_length,
        align=bde.align.ANCHOR_BOTTOM,
    )

    p -= bd.Cylinder(
        spec.main_id / 2,
        height=spec.main_length,
        align=bde.align.ANCHOR_BOTTOM,
    )

    return p


def mean(a: float, b: float) -> float:
    """Calculate avg of two numbers."""
    return (a + b) / 2


def dot_retainer_grid_printable(spec: MainSpec) -> bd.Part | bd.Compound:
    """Make a grid of dot_retainer parts."""
    single_dot_retainer = dot_retainer(spec)

    spacing = 7

    p = bd.Part(None)
    for x_val, y_val in product(
        bde.evenly_space_with_center(count=3, spacing=spacing),
        bde.evenly_space_with_center(count=3, spacing=spacing),
    ):
        p += bd.Pos((x_val, y_val, 0)) * single_dot_retainer

    # Join grids in Y.
    for x_val, y_val in product(
        bde.evenly_space_with_center(count=3, spacing=spacing),
        bde.evenly_space_with_center(count=2, spacing=spacing),
    ):
        p += bd.Pos((x_val, y_val, spec.main_length / 2)) * bd.Box(
            2, spacing - mean(spec.main_od, spec.main_id), 2
        )

    # Join grids in X.
    for x_val, y_val in product(
        bde.evenly_space_with_center(count=2, spacing=spacing),
        bde.evenly_space_with_center(count=3, spacing=spacing),
    ):
        p += bd.Pos((x_val, y_val, spec.main_length / 2)) * bd.Box(
            spacing - mean(spec.main_od, spec.main_id), 2, 2
        )

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "dot_retainer": (dot_retainer(MainSpec())),
        "dot_retainer_grid_printable": show(dot_retainer_grid_printable(MainSpec())),
    }

    logger.info("Saving CAD model(s)...")

    repo_dir = git.Repo(__file__, search_parent_directories=True).working_tree_dir
    assert repo_dir
    (export_folder := Path(repo_dir) / "build" / Path(__file__).stem).mkdir(
        exist_ok=True, parents=True
    )
    for name, part in parts.items():
        bd.export_stl(part, str(export_folder / f"{name}.stl"))
        bd.export_step(part, str(export_folder / f"{name}.step"))

    logger.info(f"Done running {py_file_name} in {datetime.now(UTC) - start_time}")
