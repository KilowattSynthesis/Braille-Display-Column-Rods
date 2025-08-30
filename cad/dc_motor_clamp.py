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

    # Motor length variable here is "motor+gearbox+shaft_lip".
    motor_length_plus_gearbox_plus_shaft_lip: float = 19.8
    motor_x_limit_thickness: float = 1.3
    motor_shaft_lip_length: float = 1.1

    # Peg settings (in corners).
    peg_sep_y: float = 9.0
    peg_od: float = 3.0
    peg_height: float = 1.0

    # Screw hole settings (in center).
    # TODO(KilowattSynthesis): Make these holes M2 on the PCB.
    screw_hole_sep_y: float = 10.0  # Shift the M2 screws toward the outsides.
    screw_d: float = 1.9  # M2 thread-forming.

    # Other core layout settings.
    hole_spacing_x: float = 5.3
    top_wall_t: float = 1.5
    total_y: float = 16.0

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {
            "total_x": self.total_x,
            "total_z": self.total_z,
            "wall_thickness_past_screw_hole": (
                # Wall thickness past the screw holes.
                # Validate: >=1.5 or >=2.0 roughly.
                (self.total_y / 2) - (self.screw_hole_sep_y / 2 + self.screw_d / 2)
            ),
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


def make_dc_motor_clamp(spec: MainSpec) -> bd.Part | bd.Compound:
    """Make the clamp for the tiny DC motor.

    Wires go out the left side.
    """
    p = bd.Part(None)

    # Draw the main body.
    main_body = bd.Part(None) + bd.Pos(
        X=-spec.motor_length_plus_gearbox_plus_shaft_lip / 2
        - spec.motor_x_limit_thickness
    ) * bd.Box(
        spec.total_x,
        spec.total_y,
        spec.total_z,
        align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN),
    )
    p += main_body.fillet(
        radius=1.6,
        # Round all edges except the bottom edges.
        edge_list=main_body.edges() - main_body.faces().sort_by(bd.Axis.Z)[0].edges(),
    )
    del main_body

    # Remove bottom half of motor.
    # Make it easy to slide the motor in.
    p -= bd.Box(
        spec.motor_length_plus_gearbox_plus_shaft_lip,
        spec.motor_od,
        spec.motor_od / 2,
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
    for y_sign in (1, -1):
        p -= bd.Pos(X=0, Y=y_sign * spec.screw_hole_sep_y / 2) * bd.Cylinder(
            radius=spec.screw_d / 2,
            height=40,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Add the pegs (in corners).
    for x_sign, y_sign in product((-1, 1), (-1, 1)):
        p += bd.Pos(
            X=x_sign * spec.hole_spacing_x, Y=y_sign * spec.peg_sep_y / 2
        ) * bd.Cylinder(
            radius=spec.peg_od / 2,
            height=spec.peg_height,
            align=bde.align.ANCHOR_TOP,
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
