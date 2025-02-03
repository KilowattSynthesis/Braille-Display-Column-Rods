"""Clamp for the tiny 6mm OD DC motor with planetary gearbox."""

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
    """Specification for the cam for the tiny DC motor."""

    motor_od: float = 6

    hole_spacing_x: float = 5.3
    hole_count_x: int = 3

    hole_spacing_y: float = 9.0

    general_length_x: float = 20

    screw_d: float = 3.25

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {}
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)


def make_dc_motor_clamp(spec: MainSpec) -> bd.Part:
    """Make the clamp for the tiny DC motor."""
    p = bd.Part(None)

    # Draw the main body.
    p += bd.Box(
        spec.general_length_x,
        spec.hole_spacing_y + 8,
        spec.motor_od + 2,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the motor body.
    p -= bd.Pos(Z=spec.motor_od / 2) * bd.Cylinder(
        spec.motor_od / 2,
        spec.general_length_x,
    ).rotate(axis=bd.Axis.Y, angle=90)

    # Remove meat.
    p -= bd.Box(
        100,
        spec.hole_spacing_y - 1.5,
        spec.motor_od / 2,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the screw holes.
    for x_value, y_value in product(
        bde.evenly_space_with_center(
            spacing=spec.hole_spacing_x, count=spec.hole_count_x
        ),
        bde.evenly_space_with_center(spacing=spec.hole_spacing_y, count=2),
    ):
        p -= bd.Pos(X=x_value, Y=y_value) * bd.Cylinder(
            radius=spec.screw_d / 2,
            height=40,
            align=bde.align.ANCHOR_BOTTOM,
        )

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "dc_motor_clamp": show(make_dc_motor_clamp(MainSpec())),
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
