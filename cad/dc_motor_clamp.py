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
    motor_plus_gearbox_length: float = 18.7

    hole_spacing_x: float = 5.3
    hole_count_x: int = 3

    hole_spacing_y: float = 9.0

    general_length_x: float = 20

    corner_screw_d: float = 2.8
    center_screw_d: float = 3.25

    bolt_head_od: float = 5.6
    bolt_head_height: float = 0.01  # Disabled, basically.

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {}
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def total_z(self) -> float:
        """Total height of the clamp."""
        return self.motor_od + 2


def make_dc_motor_clamp(spec: MainSpec) -> bd.Part:
    """Make the clamp for the tiny DC motor."""
    p = bd.Part(None)

    # Draw the main body.
    p += bd.Box(
        spec.general_length_x,
        spec.hole_spacing_y + 8,
        spec.total_z,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove meat (bottom half of motor).
    # Done in separate step to remove thin walls, and make it easy to slide
    # the motor on all the way.
    p -= bd.Box(
        spec.general_length_x,
        spec.hole_spacing_y - 1.5,
        spec.motor_od / 2,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove space for wires out the back.
    p -= bd.Pos(X=-spec.general_length_x / 2) * bd.Box(
        (spec.general_length_x - spec.motor_plus_gearbox_length),
        spec.motor_od,
        spec.motor_od / 2 + 1,
        align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN),
    )

    # Remove the motor body.
    p -= bd.Pos(
        X=(spec.general_length_x - spec.motor_plus_gearbox_length),
        Z=spec.motor_od / 2,
    ) * bd.Cylinder(
        spec.motor_od / 2,
        spec.general_length_x,
    ).rotate(axis=bd.Axis.Y, angle=90)

    # Remove the screw holes.
    for x_value, y_value in product(
        [0],
        bde.evenly_space_with_center(spacing=spec.hole_spacing_y, count=2),
    ):
        p -= bd.Pos(X=x_value, Y=y_value) * bd.Cylinder(
            radius=spec.center_screw_d / 2,
            height=40,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the bolt head for the outermost screw holes.
    for x_value, y_value in product(
        bde.evenly_space_with_center(spacing=spec.hole_spacing_x * 2, count=2),
        bde.evenly_space_with_center(spacing=spec.hole_spacing_y, count=2),
    ):
        # Remove the screw holes.
        p -= bd.Pos(X=x_value, Y=y_value) * bd.Cylinder(
            radius=spec.corner_screw_d / 2,
            height=40,
            align=bde.align.ANCHOR_BOTTOM,
        )

        # Remove the bolt head.
        p -= bd.Pos(
            X=x_value,
            Y=y_value,
            Z=spec.total_z - spec.bolt_head_height,
        ) * bd.Cylinder(
            radius=spec.bolt_head_od / 2,
            height=spec.bolt_head_height,
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
