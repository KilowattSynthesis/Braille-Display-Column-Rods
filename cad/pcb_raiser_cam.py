"""Cam for the tiny 6mm OD DC motor with planetary gearbox."""

import copy
import json
import sys
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import build123d as bd
import build123d_ease as bde
import git
from build123d_ease import show
from loguru import logger

sys.path.append(str(Path(__file__).parent))

from cad.components.tiny_dc_motor_with_gearbox import (
    MainSpec as MotorMainSpec,
)
from cad.components.tiny_dc_motor_with_gearbox import (
    make_dc_motor_and_gearbox,
)


@dataclass(kw_only=True)
class MainSpec:
    """Specification for the cam for the tiny DC motor."""

    shaft_dist_from_pcb: float = 3  # Radius of motor, basically.

    shaft_length: float = 2.5 + 1  # 2.5 nominal, plus a bit.
    shaft_od: float = 2.0
    shaft_d_size: float = 1.5  # Short dimension on D shape.

    cam_length: float = 2

    cam_travel: float = 1.2

    cam_main_od: float = 8.0

    # "zero" refers to the homing magnet holder section.
    homing_length: float = 3.0
    homing_od: float = 3.5

    magnet_od = 2.0
    magnet_depth: float = 2.0  # Nominally 1.0, but a bit extra is good.

    cam_to_block_slop: float = 1.0

    bushing_block_thickness: float = 4

    bushing_block_wall_thickness: float = 2.0
    bushing_block_slop_diameter: float = 0.5

    bushing_block_screw_hole_id: float = 1.9  # M2 thread forming screw.
    bushing_block_screw_hole_sep: float = 7.0  # Center to center.

    bushing_hall_sensor_width: float = 3.5 + 1
    bushing_hall_sensor_height: float = 0.01

    def __post_init__(self) -> None:
        """Post initialization checks."""
        data = {}
        logger.info(json.dumps(data, indent=2))

    def deep_copy(self) -> "MainSpec":
        """Copy the current spec."""
        return copy.deepcopy(self)


def make_cam(spec: MainSpec) -> bd.Part:
    """Make the cam for the tiny DC motor.

    Z=0 is the bottom of the cam (where the motor's Z=0 goes).

    The homing magnet triggers when the cam is pushing the minimal amount.
    """
    p = bd.Part(None)

    # Cam profiles: https://technologystudent.com/cams/cam2.htm

    # Draw the cam.
    p += bd.Pos(Y=-spec.cam_travel / 2) * bd.Cylinder(
        spec.cam_main_od / 2,
        height=spec.cam_length,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Add the homing magnet holder.
    p += bd.Pos(Z=spec.cam_length) * bd.Cylinder(
        spec.homing_od / 2,
        spec.homing_length + spec.bushing_block_thickness + spec.cam_to_block_slop,
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the shaft (D-Shape).
    p -= bd.Cylinder(
        spec.shaft_od / 2,
        spec.shaft_length,
        align=bde.align.ANCHOR_BOTTOM,
    ) - bd.Box(
        spec.shaft_od,
        spec.shaft_od,
        spec.shaft_length,
        align=(bd.Align.CENTER, bd.Align.MAX, bd.Align.MIN),
    ).translate((0, -(spec.shaft_od - spec.shaft_d_size), 0))

    # Remove the homing magnet.
    p -= (
        bd.Cylinder(
            radius=spec.magnet_od / 2,
            height=spec.magnet_depth,
        )
        .rotate(bd.Axis.X, angle=90)
        .translate(
            (
                0,
                -spec.homing_od / 2,
                (
                    spec.cam_length
                    + spec.bushing_block_thickness
                    + spec.cam_to_block_slop
                    + spec.homing_length / 2
                ),
            )
        )
    )

    # Remove a tiny divot for centering.
    p -= bd.Pos(
        Z=spec.cam_length
        + spec.bushing_block_thickness
        + spec.homing_length
        + spec.cam_to_block_slop
    ) * bd.Cone(
        top_radius=0.3,
        bottom_radius=0.1,
        height=0.5,
        align=bde.align.ANCHOR_TOP,
    )

    return p


def make_bushing_block(spec: MainSpec) -> bd.Part:
    """Create the bushing block for the cam.

    Motor runs in the X direction, pointing in +X.
    """
    p = bd.Part(None)

    p += bd.Box(
        spec.bushing_block_thickness,
        (
            spec.bushing_block_screw_hole_sep
            + spec.bushing_block_screw_hole_id
            + spec.bushing_block_wall_thickness * 2
        ),
        (
            # This doesn't make much sense, but it makes the part rotationally
            # symmetrical (good DFM).
            spec.shaft_dist_from_pcb * 2
        ),
        align=bde.align.ANCHOR_BOTTOM,
    )

    # Remove the hole (making it a bushing).
    p -= bd.Pos(Z=spec.shaft_dist_from_pcb) * bd.Cylinder(
        spec.homing_od / 2 + spec.bushing_block_slop_diameter / 2,
        spec.bushing_block_thickness * 5,  # Arbitrary.
    ).rotate(bd.Axis.Y, angle=90)

    for y_value in bde.evenly_space_with_center(
        spacing=spec.bushing_block_screw_hole_sep, count=2
    ):
        # Remove screw hole through bottom.
        p -= bd.Pos(Y=y_value) * bd.Cylinder(
            spec.bushing_block_screw_hole_id / 2,
            height=spec.shaft_dist_from_pcb * 3,
            align=bde.align.ANCHOR_BOTTOM,
        )

    # Remove the hall sensor hole.
    p -= bd.Box(
        20,
        spec.bushing_hall_sensor_width,
        spec.bushing_hall_sensor_height,
        align=bde.align.ANCHOR_BOTTOM,
    )

    return p


def make_assembly_cam_and_dc_motor(spec: MainSpec) -> bd.Part:
    """Make the cam and motor assembly."""
    p = bd.Part(None)

    # Add the cam.
    p += make_cam(spec)

    # Add the motor.
    p += make_dc_motor_and_gearbox(MotorMainSpec())

    return p


if __name__ == "__main__":
    start_time = datetime.now(UTC)
    py_file_name = Path(__file__).name
    logger.info(f"Running {py_file_name}")

    parts = {
        "cam": show(make_cam(MainSpec())),
        "assembly_cam_and_dc_motor": show(make_assembly_cam_and_dc_motor(MainSpec())),
        "bushing_block": show(make_bushing_block(MainSpec())),
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
