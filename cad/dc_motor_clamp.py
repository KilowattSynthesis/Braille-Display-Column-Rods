"""Clamp for the tiny 6mm OD DC motor with planetary gearbox."""

import copy
import json
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from itertools import product
from pathlib import Path

import build123d as bd
import build123d_ease as bde
import git
from build123d_ease import show
from loguru import logger

sys.path.insert(0, str(Path(__file__).parent.parent.absolute()))
from cad.components import tiny_dc_motor_with_gearbox


@dataclass(kw_only=True)
class MainSpec:
    """Specification for the cam for the tiny DC motor."""

    motor_od: float = 6

    # Motor length must be "motor+gearbox+shaft_lip"
    motor_length_plus_gearbox_plus_shaft_lip: float = 19.8
    motor_x_limit_thickness: float = 1.3
    motor_shaft_lip_length: float = 1.1

    hole_spacing_x: float = 5.3
    hole_count_x: int = 3

    hole_spacing_y: float = 9.0

    corner_screw_d: float = 2.8
    center_screw_d: float = 3.25

    bolt_head_od: float = 5.6
    bolt_head_height: float = 0.001  # Disabled, basically.

    top_wall_t: float = 1.5

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {
            "total_x": self.total_x,
            "total_z": self.total_z,
        }
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)

    @property
    def total_x(self) -> float:
        """Total length of the clamp."""
        return (
            self.motor_length_plus_gearbox_plus_shaft_lip
            + self.motor_x_limit_thickness
            - self.motor_shaft_lip_length
        )

    @property
    def total_z(self) -> float:
        """Total height of the clamp."""
        return self.motor_od + self.top_wall_t


def make_dc_motor_clamp(spec: MainSpec) -> bd.Part:
    """Make the clamp for the tiny DC motor.

    Wires go out the left side.
    """
    p = bd.Part(None)

    # Draw the main body.
    p += bd.Pos(
        X=-spec.motor_length_plus_gearbox_plus_shaft_lip / 2
        - spec.motor_x_limit_thickness
    ) * bd.Box(
        spec.total_x,
        spec.hole_spacing_y + 8,
        spec.total_z,
        align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN),
    )

    # Remove bottom half of motor.
    # Make it easy to slide the motor in.
    p -= bd.Box(
        spec.motor_length_plus_gearbox_plus_shaft_lip,
        spec.motor_od,
        spec.motor_od / 2,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove thin walls around the holes.
    for x_val in bde.evenly_space_with_center(spacing=spec.hole_spacing_x, count=3):
        p -= bd.Pos(X=x_val) * bd.Box(
            max(spec.corner_screw_d, spec.corner_screw_d) + 0.1,
            spec.hole_spacing_y,
            spec.motor_od * 0.75,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove space for wires out the back.
    p -= bd.Pos(
        X=-spec.motor_length_plus_gearbox_plus_shaft_lip / 2
        - spec.motor_x_limit_thickness
    ) * bd.Box(
        spec.motor_x_limit_thickness,
        spec.motor_od,
        spec.motor_od / 2 + 1,
        align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN),
    )

    # Remove the motor body.
    p -= bd.Pos(
        Z=spec.motor_od / 2,
    ) * bd.Cylinder(
        spec.motor_od / 2,
        height=spec.motor_length_plus_gearbox_plus_shaft_lip,
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


def make_assembly(clamp_spec: MainSpec) -> bd.Part | bd.Compound:
    """Make the assembly of the motor and the clamp."""
    p = bd.Part(None)

    # Add the clamp.
    p += make_dc_motor_clamp(clamp_spec)

    # Add the motor and gearbox.
    motor_spec = tiny_dc_motor_with_gearbox.MainSpec()
    motor_part = (
        tiny_dc_motor_with_gearbox.make_dc_motor_and_gearbox(motor_spec)
        .rotate(axis=bd.Axis.Y, angle=90)
        .rotate(axis=bd.Axis.X, angle=90)
        .translate(
            (
                motor_spec.motor_plus_gearbox_plus_shaft_lip / 2 + 0.1,
                0,
                clamp_spec.motor_od / 2 - 0.1,
            )
        )
    )
    motor_part.color = bd.Color("red", alpha=0.5)
    p += motor_part

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "dc_motor_clamp": show(make_dc_motor_clamp(MainSpec())),
        "dc_motor_clamp_assembly": show(make_assembly(MainSpec())),
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
